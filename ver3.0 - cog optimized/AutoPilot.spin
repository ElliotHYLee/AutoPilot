CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

  ULTRASONIC_SENSOR_PIN = 8
                      
OBJ

  comm           : "fullDuplexSerial4Port_tier3.spin"
  sensor         : "tier3MPUMPL_DCM.spin"
  motors         : "Motors.spin"
  math           : "MyMath.spin"
  attCtrl        : "PID_Attitude.spin"
  heightCtrl     : "PID_Height.spin"
  ping           : "ping.spin"
VAR
  'system variable
    
  'systemMode 1 = idle     (idel pwm: 1100)
  'systemMode 2 = prepare  (default pwm: 1200)
  'systemMode 3 = hover    (let PID control pwm to maintain current balance)
  'systemMode 4 = navigate (let PID control pwm)  
  'long systemMode, respondType, respondContent

  
  'motor variables
  long throttle, pulse[6], motorPin[6], motorStack[128], motorCogId 

 'attitude variables
  long sensorCodId, sensorStack[128] 
  long gyro[3], acc[3], eAngle[3],mag[3]

  'communcation variables
  long commStack[128],comm_listener_CogId, comm_loop_CogId


  'pid variables  - attitude control
  long pidStack[128], pidCogId
  long targetEAngle[3], pidUpdateIndex
  long xKp, xKd, xKi 
  long yKp, yKd, yKi
  long zKp, zKd, zKi  
  long xErr, xPro, xDer, xInt, xOutput
  long yErr, yPro, yDer, yInt, yOutput
  long zErr, zPro, zDer, zInt, zOutput
  long pidOnOff[3]
  long thrustBound_max



  'navigation pid variables - position control
  long pidStack_pos[128], pidCodId_pos, navPidOnOff[3]
  
  
    'distance sensor var
  long dist_ground, filtered_dist, raw_dist, target_dist

    'local cooridinate
  long localCoord[3]
    
  
   
  long prevTime, curTime, tElapse, dummy , dummy2 , sensorElapse

PUB startAutoPilot|i

  repeat i from 0 to 2
    targetEAngle[i] := 0

  '1. xbee start (wireless com for Ground Station)      x 2 cogs
  newCommunication
  'usb.quickStart
  
  '2. attitude start (MPU9150(+AK8) & MPL11A2)          x 2 cog
  startSensor
  
  '4. attitude pid start                                x 1 cog
  startPID

  '5. Position PID                                      x 1 cog
  startPID_Pos 

  '6. motor start                                       x 1 cog
  setMotor(2,3,4,5,6,7)

  '7 sd card
  'cogstop(0)                        

'===================================================================================================
'===================== COMMUNICATION PART ==================================================================
'===================================================================================================
{{
-----------------------------------------------------------------
USB REGION                                                      |
  Number of cog used : 2                                        |
  Cog usage          : sending/reading data via usb & xbee      |
  Functions:         :                                          |
-----------------------------------------------------------------
}}

PUB newCommunication

  comm_listener_CogId := comm.initialize

  comm.setLocalCoordinate(@localCoord)
  comm.setAttPtr(@acc, @gyro, @eAngle)
  comm.setMotPtr(@pulse)
  comm.setThrottle(@throttle)
  comm.setXPidPtr(@xKp, @xKd, @xKi, @xPro, @xDer, @xInt, @xOutput)
  comm.setYPidPtr(@yKp, @yKd, @yKi, @yPro, @yDer, @yInt, @yOutput)
  comm.setZPidPtr(@zKp, @zKd, @zKi, @zPro, @zDer, @zInt, @zOutput)
  comm.setPidOnOffPtr(@pidOnOff)
  comm.setTargetAttitude(@targetEAngle)
  comm.setDistPtr(@dist_ground)
  comm.setNavPidOnOffPtr(@navPidOnOff)

  startCommunication

PRI stopCommunication
  if comm_loop_CogId
    cogstop(comm_loop_CogId)

PRI startCommunication  

  stopCommunication
  comm_loop_CogId := cognew(runCommunication, @commStack)
  
PRI runCommunication

  comm.communicate 

'===================================================================================================
'===================== Navigation PID PART       ===================================================
'===================================================================================================
{{
-----------------------------------------------------------------
PID REGION                                                      |
  Number of cog used : 1                                        |
  Cog usage          : calculating PID _ Navigation             |
  Functions:         :                                          |
-----------------------------------------------------------------
}}

PRI navPidOnX
  navPidOnOff[0] := 1
  
PRI navPidOffX
  navPidOnOff[0] := 0

PRI navPidOnY
  navPidOnOff[1] := 1
  
PRI navPidOffY
  navPidOnOff[1] := 0

PRI navPidOnZ
  navPidOnOff[2] := 1
  
PRI navPidOffZ
  navPidOnOff[2] := 0  

PRI navPidOn
  navPidOnX
  navPidOnY
  navPidOnZ

PRI navPidOff
  navPidOffX  
  navPidOffY
  navPidOffZ
    
PUB stopPID_Pos
  if pidCodId_pos             
    cogstop(pidCodId_pos ~ - 1)
PUB startPID_Pos

  stopPID_pos

  navPidOff
  pidCodId_pos := cognew(runPID_pos, @pidStack_pos) + 1  'start running pid controller

PUB runPID_pos | base, val, diff, totalInc, timeElapse

  totalInc := 0
  base := cnt
  repeat
    getDistance_Ground
    if (navPidOnOff[2])
      val := heightCtrl.calculateThrottle(dist_ground, 500, cnt - base)
      diff := val - throttle ' positive difference when need to go up, negetive when need to go down
      
      if (diff > 0)

        if(diff >100)
          throttle :=  throttle + 100
          waitcnt(cnt + clkfreq)
        else
          totalInc += diff
          timeElapse := cnt - base
          if (timeElapse < clkfreq)
            if (totalInc > 100)
              totalInc := 0                                         
              waitcnt(cnt + clkfreq)
          throttle := throttle + diff    
      elseif (diff < 0)

        if(diff <-100)
          throttle :=  throttle - 100
          waitcnt(cnt + clkfreq)
        else
          totalInc += diff
          timeElapse := cnt - base
          if (timeElapse < clkfreq)
            if (totalInc < -100)
              totalInc := 0                                         
              waitcnt(cnt + clkfreq)
          throttle := throttle - diff 
      
    base:=cnt

PUB getDistance_Ground
  if (  (getAbs(eAngle[0]) < 500) AND (getAbs(eAngle[1]) <500) )
    raw_dist := ping.Millimeters(ULTRASONIC_SENSOR_PIN)
    if (raw_dist>300)
      'low pass filter
      filtered_dist := raw_dist*5/100 + filtered_dist *95/100
      'tilt angle adjustment + low pass  
      dist_ground := raw_dist'getAbs(acc[2])*raw_dist / 16800'sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2])
     
PUB getAbs(value)
  if value > 0
    result := value
  else
    result := -value
'=====================

PUB sqrt(value)| x, i

  x := value

  repeat i from 0 to 20
    x := (value/x + x) /2

  return x

'===================================================================================================
'===================== PID PART Attitude Control===================================================
'===================================================================================================
{{
-----------------------------------------------------------------
PID REGION                                                      |
  Number of cog used : 1                                        |
  Cog usage          : calculating PID                          |
  Functions:         : pidOn                                    |
                       pidOff                                   |
                       stopPID                                  |
                       startPID                                 |
                       runPID                                   |
                       pidAxis                                  |
-----------------------------------------------------------------
}}

PRI pidOnX
  pidOnOff[0] := 1
  
PRI pidOffX
  pidOnOff[0] := 0

PRI pidOnY
  pidOnOff[1] := 1
  
PRI pidOffY
  pidOnOff[1] := 0

PRI pidOnZ
  pidOnOff[2] := 1
  
PRI pidOffZ
  pidOnOff[2] := 0  

PRI stopPID
  if pidCogId             
    cogstop(pidCogId ~ - 1)

PRI startPID

  stopPID

  pidCogId := cognew(runPID, @pidStack) + 1  'start running pid controller

PRI setXConst  | x   'Roll

  x := throttle

  xKp := 500
  xKi := 1500
  xKd := 800     

PRI setYConst  | x    ' pitch

  x := throttle

  yKp := 400
  yKi := 1500
  yKd := 600    

PRI setZConst  | x

  x := throttle

  zKp := 150
  zKi := 1000
  zKd := 1000

PRI runPID  |i, prev, dt, delay

  
  setXConst
  setYConst
  setZConst

  pidOffX
  pidOffY
  pidOffZ

  {
  respondContent := 2
  respondType := 1
  respondContent := 1
  respondType := 1       
   }
  attCtrl.setXaxis(@xKp, @xKd, @xKi)
  attCtrl.setYaxis(@yKp, @yKd, @yKi)
  attCtrl.setZaxis(@zKp, @zKd, @zKi)  
  
  attCtrl.setAttVal(@eAngle, @gyro)

  thrustBound_max := 1950
  repeat
    prev := cnt
    
    updateAttitude   

    attitudePID
    
    dt := cnt - prev

    waitcnt(cnt + clkfreq/100 - dt   )     ' 137 = ~avg DCM frequency
    
PRI updateAttitude 

  sensor.getMag(@mag)
  sensor.getAcc(@Acc)
  sensor.getGyro(@gyro)
  sensor.getEulerAngles(@eAngle)

PRI attitudePID

  xOutput:=0
  yOutput:=0
  zOutput:=0
  
  if pidOnOff[0] == 1
    rollPID
  else
    attCtrl.resetX
    
  if pidOnOff[1] == 1
    pitchPID
  else
    attCtrl.resetY

  if pidOnOff[2] == 1
    yawPID
  else
    attCtrl.resetZ

  if pidOnOff[0] OR pidOnOff[1] OR pidOnOff[2]
    pulse[0] := 1200 #> throttle - xOutput*86/100 + yOutput - zOutput <# thrustBound_max'1950    
    pulse[1] := 1200 #> throttle + xOutput*86/100 + yOutput + zOutput <# thrustBound_max'1950
    pulse[2] := 1200 #> throttle + xOutput*86/100                     <# thrustBound_max'1950 
    pulse[3] := 1200 #> throttle + xOutput*86/100 - yOutput - zOutput <# thrustBound_max'1950
    pulse[4] := 1200 #> throttle - xOutput*86/100 - yOutput + zOutput <# thrustBound_max'1950 
    pulse[5] := 1200 #> throttle - xOutput*86/100                     <# thrustBound_max'1950 
    
     
  
                                                                                 
       
PRI rollPID 

  'setXConst

  xOutput := attCtrl.calcPIDRoll(targetEAngle[1]) 
  xErr := attCtrl.getErrX
  xPro := attCtrl.getProX
  xDer := attCtrl.getDerX
  xInt := attCtrl.getIntX

PRI pitchPID 'y = pitch axis

  'setYConst

  yOutput := attCtrl.calcPIDPitch(targetEAngle[0]) 
  yErr := attCtrl.getErrY
  yPro := attCtrl.getProY
  yDer := attCtrl.getDerY
  yInt := attCtrl.getIntY
  {
  pulse[0] := 1200 #> throttle + yOutput <# thrustBound_max'1950
  pulse[1] := 1200 #> throttle + yOutput <# thrustBound_max'1950  
  pulse[3] := 1200 #> throttle - yOutput <# thrustBound_max'1950
  pulse[4] := 1200 #> throttle - yOutput <# thrustBound_max'1950
  }
PRI yawPID

  'setZConst

  zOutput := attCtrl.calcPIDYaw(targetEAngle[2]) 
  zErr := attCtrl.getErrZ
  zPro := attCtrl.getProZ
  zDer := attCtrl.getDerZ
  zInt := attCtrl.getIntZ
  {
  pulse[0] := 1200 #> throttle - zOutput <# thrustBound_max'1950 
  pulse[3] := 1200 #> throttle - zOutput <# thrustBound_max'1950  

  pulse[1] := 1200 #> throttle + zOutput <# thrustBound_max'1950    
  pulse[4] := 1200 #> throttle + zOutput <# thrustBound_max'1950 
  }

  

                 
'===================================================================================================
'===================== MOTOR PART ==================================================================
'===================================================================================================
{{
-----------------------------------------------------------------
MOTOR CONTROL REGION                                            |
  Number of cog used : 1                                        |
  Motors             : Brushless DC motor x 6                   |
  Cog usage          : Generating PWM for motor                 |
  Functions:         : newMotor                                 |
                       startMotor                               |
                       stopMotor                                |
                       initMotor                                |
                       runMotor                                 |
                       insepctPulse                             |
-----------------------------------------------------------------
}}

PRI setThrottle(value)
  throttle := value

PRI setMotor(pin0, pin1, pin2, pin3, pin4, pin5)  {{ constructor }}
  motors.setMotorPins(pin0, pin1, pin2, pin3, pin4, pin5)
  setThrottle(1200)
  
  motors.setMotorPWM(@pulse)
  startMotor

PRI startMotor
  stopMotor
  motorCogId := cognew(runMotor, @motorStack) + 1  'start running motor

PRI stopMotor {{kind of destructor}}
  if motorCogId
    cogstop(motorCogId ~ - 1)

PRI runMotor

  motors.runMotor   ' this funcion has a loop in it

'===================================================================================================
'===================== ATTITUDE SENSOR PART ==================================================================
'===================================================================================================
{{
-----------------------------------------------------------------
ATTITUDE SENSOR REGION                                          |
  Number of cog used : 2                                        |
  Sensors            : MPU9150, AK8, MPL                        |
  Cog usage          : Reading Sensor Values                    |
                       Calculating Complementary Filter         |
  Functions:         : stopSensor                               |
                       startSensor (call startSensor)           |
                       runSensor (start MPU, AK8, MPl sensor)   |
-----------------------------------------------------------------
}}

PRI stopSensor
  if sensorCodId
    cogstop(sensorCodId ~ - 1)
  
PRI startSensor 
'  sensor.initSensor(15,14) ' scl, sda, cFilter portion in %
'  sensor.setMpu(%000_11_000, %000_01_000) '2000deg/s, 4g
'  stopSensor
'  sensorCodId:= cognew(runSensor, @sensorStack) + 1

sensor.masterKey_tier3

{
PRI runSensor
  repeat
    'dummy := cnt
    sensor.run
    'dummy2 := cnt
    'sensorElapse := (cnt*100 - dummy*100)/clkfreq
}  