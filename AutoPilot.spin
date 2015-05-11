CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
                      
OBJ
  usb            : "Parallax Serial Terminal"
  xbee           : "Communication_XBee"
  sensor         : "tier2MPUMPL.spin"
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
  long throttle, pulse[6], motorPin[6], motorStack[128],motorCogId 
  byte motorIteration 

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
  
  long prevTime, curTime, tElapse, dummy , dummy2 , sensorElapse
  byte pidOnOff[3]

PUB startAutoPilot|i

  repeat i from 0 to 2
    targetEAngle[i] := 0

  '1. usb start  (usb com for Kinect)                   x 2 cogs
  'newUSB

  '2. xbee start (wireless com for Ground Station)      x 2 cogs
  newXBee
  
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

PRI runPID  |i

  xKp := 100
  xKi := 3050
  xKd := 250
  
  yKp := 120
  yKi := 0
  yKd := 250
 
  pidOffX
  pidOffY
  pidOffZ
  
  respondContent := 2
  respondType := 1
  respondContent := 1
  respondType := 1              

  attCtrl.setXaxis(@xKp, @xKd, @xKi)
  attCtrl.setYaxis(@yKp, @yKd, @yKi)
  
  attCtrl.setAttVal(@eAngle, @gyro)
  
  repeat
    sensor.getEulerAngle(@eAngle)
    sensor.getAcc(@acc)
    sensor.getGyro(@gyro)
    if pidOnOff[0] == 1
      xAxisPID
    if pidOnOff[1] == 1
      yAxisPID
    if pidOnOff[2] == 1
      'zAxisPID 
       
PRI xAxisPID

  xOutput := attCtrl.calcPIDx(targetEAngle[0]) 
  xErr := attCtrl.getErr
  xPro := attCtrl.getPro
  xDer := attCtrl.getDer
  xInt := attCtrl.getInt
  pulse[3] := 1200 #> 1300 - xOutput <# 1600
  pulse[1] := 1200 #> 1300 + xOutput <# 1600

       
PRI yAxisPID

  yOutput := attCtrl.calcPIDy(targetEAngle[0]) 
  yErr := attCtrl.getErr
  yPro := attCtrl.getPro
  yDer := attCtrl.getDer
  yInt := attCtrl.getInt
  pulse[0] := 1200 #> 1300 - yOutput <# 1600
  pulse[2] := 1200 #> 1300 + yOutput <# 1600


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
'===================== USB COMMUNICATION PART ==================================================================
'===================================================================================================
{{
-----------------------------------------------------------------
USB REGION                                                      |
  Number of cog used : 2                                        |
  Cog usage          : sending/reading data via usb & xbee      |
  Functions:         :                                          |
-----------------------------------------------------------------
}}

PUB newUSB

  serialCogId1 := usb.start(115200)
  
  startUSB

PRI stopUSB
  if comCogId
    cogstop(comCogId ~ - 1)
  
PRI startUSB
  stopUSB
  respondType := 0      
  comCogId := cognew(communicate, @comStack) + 1  'start running motor

PRI communicate | base
  base := cnt  
  repeat
    if usb.RxCount > 0  
      readCharArray
    else
      if respondType > 0 ' need to respond to the request from C#
        respondBack(respondType)
        
      else
        if (cnt > base + clkfreq/90)
          sendOrdinaryMsg
          base := cnt
          'sendXbeeMsg
          'sendTestMsg
          'sendPidTestMsg
  
PRI sendTestMsg
  usb.clear   

  usb.str(String("[ax"))
  usb.dec(acc[0])
  usb.str(String("]  "))

  usb.str(String("[cx"))
  usb.dec(eAngle[0])
  usb.strLn(String("]"))
  
  usb.str(String("[ay"))
  usb.dec(acc[1])
  usb.str(String("]"))

  usb.str(String("  [cy"))
  usb.dec(eAngle[1])
  usb.strLn(String("]"))  

  usb.str(String("[az"))
  usb.dec(acc[2])
  usb.str(String("]"))  

  usb.str(String("  [cz"))
  usb.dec(eAngle[2])
  usb.strLn(String("]"))  

  waitcnt(cnt + clkfreq/10)        
 
PRI sendOrdinaryMsg | i  

  usb.str(String("[k0"))
  usb.dec(xDer)
  usb.str(String("]"))
  usb.str(String("[k1"))
  usb.dec(xPro)
  usb.str(String("]"))
  usb.str(String("[k3"))
  usb.dec(xOutput)
  usb.str(String("]"))

 'write motor info
  i:=0                 
  repeat while i < 4
    'motor write
    usb.str(String("[m"))
    usb.Dec(i+1)
    usb.Dec(pulse[i])
    usb.str(String("]"))

   'eAngle write
    if i < 3 
      usb.str(String("[c"))
      case i
        0: usb.str(String("x"))
        1: usb.str(String("y"))
        2: usb.str(String("z"))
      usb.dec(eAngle[i])
      usb.str(String("]"))
      {
      usb.str(String("[a"))
      case i
        0: usb.str(String("x"))
        1: usb.str(String("y"))
        2: usb.str(String("z"))
      usb.dec(acc[i])
      usb.str(String("]"))

      usb.str(String("[g"))
      case i
        0: usb.str(String("x"))
        1: usb.str(String("y"))
        2: usb.str(String("z")) 
      usb.dec(gyro[i])
      usb.str(String("]"))
      }
    i++
            


                  
PRI respondBack(x)
  case x
    1:
      if respondContent == 1     ' respondContent type 1 = pid gains
        usb.str(String("[pp"))
        usb.dec(xkp)
        usb.str(String("]"))
        usb.str(String("[pi"))
        usb.dec(xki)
        usb.str(String("]"))
        usb.str(String("[pd"))
        usb.dec(xkd)
        usb.str(String("]"))               
      elseif respondContent == 2    ' respondContent type 1 = pid on/off status 
        usb.str(String("[po"))
        usb.dec(pidOnOff)
        usb.str(String("]"))  

  respondType := 0
  respondContent := 0


PRI char2ASCII(charVar)  ' currently not used
  result := byte[charVar]
  ' Don't know how, but this returns ascii code of char

PRI ASCII2Dec(ASCII)
  result := ASCII - 48    
  
PRI readCharArray   | newPWM, newPidProperty, newRequest, newMode
   varChar := usb.CharIn
   if (48=<varChar AND varChar=<57) 'btw 0-9
     newValue := newValue*10 + ASCII2Dec(varChar)
   elseif(varChar == 77) ' M -> motor
     type := 1  'next 5 digits are (motornumber & pwm)
   elseif(varChar == 80) ' P -> PID
     type := 2  'next 5 digits are (PID type and info)
   elseif(varChar == 82) ' R -> request of specific info
     type := 3  ' next 2 digits are request types
   elseif(varChar == 68) ' D -> system mode update
     type := 4  ' next 5 digits are mode types
                       
   if (type==1)
     if 11099 < newValue AND newValue < 43000
       motorNumber := newValue/10000
       newPWM := newValue//10000
       case motorNumber
         1: pulse[0] := newPWM
         2: pulse[1] := newPWM  
         3: pulse[2] := newPWM  
         4: pulse[3] := newPWM
       type := 0
       newValue := 0
   elseif (type == 2)   ' PID constant update
     if 9_999_999 < newValue
       pidUpdateIndex := newValue/10_000_000
       newPidProperty := newValue//10_000_000
       'waitcnt(cnt + clkfreq*5)
       case pidUpdateIndex
         1: pidOnOff := newPidProperty
         2: xKp := newPidProperty
         3: xki := newPidProperty
         4: xkd := newPidProperty
       type := 0
       newValue := 0
       respondContent := 1   ' respond content 1 = pid constants
       respondBack(1)         'repond type 1 = all pid types
       respondContent := 2   ' respond content 2 = pid on/off
       respondBack(1)         'repond type 1 = all pid types

       
   elseif (type == 3)  ' Request system information
     if 10 < newValue
       respondType := newValue/10
       newRequest := newValue//10
       'waitcnt(cnt + clkfreq*5)
       case respondType
         1: respondContent := newRequest
       type := 0
       newValue := 0
       usb.RxFlush
       
   elseif (type ==4)    ' systemMode update
     if 10000 < newValue
       newMode := newValue//10000
       systemModeUpdate(newMode)
       usb.RxFlush
       newValue := 0
       respondContent := 2 'respond content 2 = pid on/off
       respondBack(1)    'repond type 1 = all pid types  

PRI systemModeUpdate(mode)

  systemMode := mode
  case mode
     1: 'idle
       pidOffX
       pidOffY
       'pidOffZ
       pulse[0] := 1100
       pulse[1] := 1100
       pulse[2] := 1100
       pulse[3] := 1100
     2: 'prepare
       pidOnX
       pidOnY
       'pidOnZ
       pulse[0] := 1200
       pulse[1] := 1200
       pulse[2] := 1200
       pulse[3] := 1200
     3: 'hover
       pidOnX
       pidOnY
       'pidOnZ
     4: 'navigation
       pidOnX
       pidOnY
       'pidOnZ


'===================================================================================================
'===================== MOTOR PART ==================================================================
'===================================================================================================
{{
-----------------------------------------------------------------
MOTOR CONTROL REGION                                            |
  Number of cog used : 1                                        |
  Motors             : Brushless DC motor x 4                   |
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
  sensor.initSensor(15,14) ' scl, sda, cFilter portion in %
  sensor.setMpu(%000_11_000, %000_01_000) '2000deg/s, 4g
  stopSensor
  sensorCodId:= cognew(runSensor, @sensorStack) + 1

PRI runSensor
  repeat
    dummy := cnt
    sensor.run
    dummy2 := cnt
    sensorElapse := (cnt*100 - dummy*100)/clkfreq

    