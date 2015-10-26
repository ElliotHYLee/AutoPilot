CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
                      
OBJ
  usb            : "ParallaxSerialTerminal"
  xbee           : "Communication_XBee"
  sensor         : "tier3MPUMPL_DCM.spin"
  motors         : "Motors.spin"
  math           : "MyMath.spin"
  attCtrl        : "PID_Attitude.spin"
  
VAR
  'system variable
    
  'systemMode 1 = idle     (idel pwm: 1100)
  'systemMode 2 = prepare  (default pwm: 1200)
  'systemMode 3 = hover    (let PID control pwm to maintain current balance)
  'systemMode 4 = navigate (let PID control pwm)  
  long systemMode, respondType, respondContent
  
  'motor variables
  long throttle, pulse[6], motorPin[6], motorStack[128], motorCogId 

 'attitude variables
  long sensorCodId, sensorStack[128] 
  long gyro[3], acc[3], eAngle[3]

  'usb variables
  long newValue, type, comStack[64],comCogId, serialCogId1
  long varChar, motorNumber

  'Xbee variables
  long serialCogId_XBee, comCogId_XBee, comStack_XBee[128]

  'pid variables
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

   
  long prevTime, curTime, tElapse, dummy , dummy2 , sensorElapse

PUB startAutoPilot|i

  repeat i from 0 to 2
    targetEAngle[i] := 0

  '1. usb start  (usb com for Kinect)                   x 2 cogs
  'newUSB

  '2. xbee start (wireless com for Ground Station)      x 2 cogs
  newXBee
  'usb.quickStart
  
  
  '3. attitude start (MPU9150(+AK8) & MPL11A2)          x 1 cog
  startSensor
  
  '4. attitude pid start                                x 1 cog
  startPID
  'position PID

  '6. motor start                                       x 1 cog
  setMotor(2,3,4,5,6,7)

  cogstop(0)                                      '-------------------
                                                  ' total : 8 cogs

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

PRI setXConst  | x

  x := throttle

  xKp := 900
  xKi := 2000
  xKd := 999     

PRI setYConst  | x    ' pitch

  x := throttle

  yKp := 800
  yKi := 0
  yKd := 990    

PRI setZConst  | x

  x := throttle

  zKp := 50
  zKi := 0
  zKd := 10 

PRI runPID  |i, prev, dt, delay

  
  setXConst
  setYConst
  setZConst

  pidOffX
  pidOffY
  pidOffZ
  
  respondContent := 2
  respondType := 1
  respondContent := 1
  respondType := 1       

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

  'sensor.getMag(@mag)
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
    pulse[1] := 1200 #> throttle + xOutput*86/100 + yOutput + zOutput <# thrustBound_max'1950
    pulse[2] := 1200 #> throttle + xOutput*86/100                     <# thrustBound_max'1950 
    pulse[3] := 1200 #> throttle + xOutput*86/100 - yOutput - zOutput <# thrustBound_max'1950    
     
    pulse[4] := 1200 #> throttle - xOutput*86/100 - yOutput + zOutput <# thrustBound_max'1950 
    pulse[5] := 1200 #> throttle - xOutput*86/100                     <# thrustBound_max'1950 
    pulse[0] := 1200 #> throttle - xOutput*86/100 + yOutput - zOutput <# thrustBound_max'1950
     
  
                                                                                 
       
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
'===================== XBee COMMUNICATION PART ==================================================================
'===================================================================================================
{{
-----------------------------------------------------------------
USB REGION                                                      |
  Number of cog used : 2                                        |
  Cog usage          : sending/reading data via usb & xbee      |
  Functions:         :                                          |
-----------------------------------------------------------------
}}

PUB newXBee

  ' assume it's xbee
  serialCogId_Xbee := xbee.init(31,30,0,115200)
  'serialCogId_Xbee := xbee.init(0,1,0,57600)  

  xbee.setAttPtr(@acc, @gyro, @eAngle)
  xbee.setMotPtr(@pulse)
  xbee.setThrottle(@throttle)
  xbee.setXPidPtr(@xKp, @xKd, @xKi, @xPro, @xDer, @xInt, @xOutput)
  xbee.setYPidPtr(@yKp, @yKd, @yKi, @yPro, @yDer, @yInt, @yOutput)
  xbee.setZPidPtr(@zKp, @zKd, @zKi, @zPro, @zDer, @zInt, @zOutput)
  xbee.setPidOnOffPtr(@pidOnOff)   
  startXBee

PRI stopXBee
  if comCogId_Xbee
    cogstop(comCogId_Xbee)

PRI startXBee

  stopXBee
  comCogId_XBee := cognew(runXBee, @comStack_XBee)
  
PRI runXBee

  xbee.communicate 
                 
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
  Number of cog used : 1                                        |
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