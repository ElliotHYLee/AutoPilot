CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
                      
OBJ
  usb            : "Parallax Serial Terminal"
  fNum           : "FloatMath.spin"
  sensor         : "tier2MPUMPL.spin"
  fStr           : "FloatString.spin"
VAR
  'system variable

  'systemMode 1 = idle     (idel pwm: 1100)
  'systemMode 2 = prepare  (default pwm: 1200)
  'systemMode 3 = hover    (let PID control pwm to maintain current balance)
  'systemMode 4 = navigate (let PID control pwm)  
  long systemMode, respondType, respondContent
  
  'motor variables
  long pulse[4], motorPin[4], motorStack[64],motorCogId 
  byte motorIteration 

 'attitude variables
  long sensorCodId, sensorStack[128] 
  long gyro[3], acc[3], eAngle[3], targetEAngle[3] 

  'usb variables
  long newValue, type, usbStack[64],usbCogId, pstCodId
  long varChar, motorNumber 

  'pid variables
  long pidStack[128], pidCogId
  long targetEAnlge10E5[3] , fProportional, fIntegral, fDerivative
  long kp, ki, kd, pidUpdateIndex, prevTime[2] , fErrorPrev[2], error, dError, dt, proportional, output
  byte pidOnOff

PUB startAutoPilot


  'usb start
  newUSB

 'attitude start
  startSensor
  
  'motor start
  newMotor(0,1,2,3)

  waitcnt(cnt + clkfreq*3)
  'pid start
  startPID

  cogstop(0)


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
    sensor.run

'===================================================================================================
'===================== PID PART ==================================================================
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

PRI pidOn
  pidOnOff := 1
  
PRI pidOff
  pidOnOff := 0

PRI stopPID
  if pidCogId
    cogstop(pidCogId ~ - 1)

PRI startPID

  stopPID

  pidCogId := cognew(runPID, @pidStack) + 1  'start running pid controller

PRI runPID  |i

  kp := 1000
  ki := 0
  kd := 0

  'pidOff  
  respondContent := 2
  respondType := 1
  respondContent := 1
  respondType := 1              

  repeat
    sensor.getEulerAngle(@eAngle)
'    sensor.getAcc(@acc)
    sensor.getGyro(@gyro)
    if pidOnOff == 1
      pidAxis(0,2) ' x axis pid set ( white arms of the drone)
      'pidAxis(1,3) ' y axis pid s0et ( red arms of the drone)  
    else
      'Do nothing  

PRI pidAxis(nMoter, pMoter)| axis, roundAdj, roundBy

  if nMoter == 0         'for x axis 
    axis := 0
  else                   'for y axis 
    axis := 1

  error := (targetEAngle[axis] - eAngle[axis])/100
  
  if error > 0
    'proportional := (kp*error + 50)/100
    error := (error + 5) / 10
  else
    'proportional := (kp*error - 50)/100 
    error := (error - 5) / 10   

  proportional := error*kp/1000
  
  outPut := proportional          
  'usb.dec(outPut)
  'usb.newline
  if eAngle[axis] > 0  ' when tilted to positive x axis - increase motor 3 , or 4 for positive y axis
    if pulse[pMoter] + (-outPut)  =< 1500
      pulse[pMoter] := pulse[pMoter] + (-outPut)   
    if (pulse[nMoter] - (-outPut)) => 1300
      pulse[nMoter] := pulse[nMoter] - (-outPut)
  elseif eAngle[axis] < 0  ' when tilted to negative x axis - increase motor 1, or 2 negative y axis
    if pulse[nMoter] + (outPut) =< 1550
      pulse[nMoter] := pulse[nMoter] + (outPut) 
    if (pulse[pMoter] - (outPut)) => 1300     
      pulse[pMoter] := pulse[pMoter] - (outPut)  

'===================================================================================================
'===================== COMMUNICATION PART ==================================================================
'===================================================================================================
{{
-----------------------------------------------------------------
PID REGION                                                      |
  Number of cog used : 1                                        |
  Cog usage          : sending/reading data via usb             |
  Functions:         :                                          |
-----------------------------------------------------------------
}}

PRI newUSB
  pstCodId:=usb.start(115200)
  '------------------
  startUSB
  '------------------  
PRI stopUSB
  if usbCogId
    cogstop(usbCogId ~ - 1)
  
PRI startUSB
  stopUSB
  respondType := 0      
  usbCogId := cognew(communicate, @usbStack) + 1  'start running motor

PRI communicate 
  repeat
    if usb.RxCount > 0  
      readCharArray
    else
      if respondType > 0 ' need to respond to the request from C#
        respondBack(respondType)
      else
        sendOrdinaryMsg

PRI respondBack(x)
  case x
    1:
      if respondContent == 1     ' respondContent type 1 = pid gains
        usb.str(String("[pp"))
        usb.dec(kp)
        usb.str(String("]"))
        usb.str(String("[pi"))
        usb.dec(ki)
        usb.str(String("]"))
        usb.str(String("[pd"))
        usb.dec(kd)
        usb.str(String("]"))               
      elseif respondContent ==2    ' respondContent type 1 = pid on/off status 
        usb.str(String("[po"))
        usb.dec(pidOnOff)
        usb.str(String("]"))  

  respondType := 0
  respondContent := 0
 
PRI sendOrdinaryMsg | i  

  usb.str(String("[k0"))
  usb.dec(error)
  usb.str(String("]"))
  usb.str(String("[k1"))
  usb.dec(proportional)
  usb.str(String("]"))
  usb.str(String("[k3"))
  'usb.dec(derivative)
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
    i++
      
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
         2: kp := newPidProperty
         3: ki := newPidProperty
         4: kd := newPidProperty
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
       pidOff
       pulse[0] := 1100
       pulse[1] := 1100
       pulse[2] := 1100
       pulse[3] := 1100
     2: 'prepare
       pidOn
       pulse[0] := 1200
       pulse[1] := 1200
       pulse[2] := 1200
       pulse[3] := 1200
     3: 'hover
       pidOn
     4: 'navigation
       pidOn




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
PRI newMotor(pin0, pin1, pin2, pin3)  {{ constructor }}
  motorPin[0] := pin3  'set pin number for this motor
  motorPin[1] := pin1
  motorPin[2] := pin2
  motorPin[3] := pin0
  'waitcnt(cnt + clkfreq)
  startMotor
  
PRI startMotor
  stopMotor
  motorCogId := cognew(runMotor, @motorStack) + 1  'start running motor

PRI stopMotor {{kind of destructor}}
  if motorCogId
    cogstop(motorCogId ~ - 1)

PRI initMotor                                             {{initializing the motor connected to this pin}}
  motorIteration:=0                       'set pin directions               
  repeat while motorIteration<4
    dira[motorPin[motorIteration]] := 1
    pulse[motorIteration] :=45
    motorIteration++  
  
  repeat while pulse[0] < 150
    motorIteration:=0  
    repeat while motorIteration<4
      outa[motorPin[motorIteration]]:=1
      waitcnt(cnt + (clkfreq / 1000 ) )
      outa[motorPin[motorIteration]]:=0
      pulse[motorIteration] ++
      motorIteration++
    waitcnt(cnt + clkfreq / 1000*20)

PRI runMotor | check, baseTime, totalElapse                 {{generating pwm for the motor connected to this pin}}              
  
  initMotor  'physical initialization for this motor 
  motorIteration := 0
  repeat while motorIteration<4
    dira[motorPin[motorIteration]] := 1   'set pin direction for this motor 
    pulse[motorIteration] := 1200         'set default pwm
    motorIteration++
  
  repeat
    check := inspectPulse
    if check==0 'abnormaly
      'usb.str(String("running well"))
       totalElapse:=0
       baseTime := cnt    
       
       outa[motorPin[0]]:= 1
       waitcnt(baseTime + clkfreq/1000000*1150)
       outa[motorPin[0]]:= 0
         
       outa[motorPin[1]]:= 1 
       waitcnt(cnt + clkfreq/1000000*1150)
       outa[motorPin[1]]:= 0
       
       outa[motorPin[2]]:= 1
       waitcnt(cnt + clkfreq/1000000*1150)
       outa[motorPin[2]]:= 0
        
       outa[motorPin[3]]:= 1
       waitcnt(cnt + clkfreq/1000000*1150)
       outa[motorPin[3]]:= 0
       totalElapse := pulse[0] + pulse[1] + pulse[2] + pulse[3]
       waitcnt(baseTime + (clkfreq/1000*20 - clkfreq/1000000*totalElapse))      

    else   ' good to go
    
      'usb.str(String("running well"))
       totalElapse:=0
       baseTime := cnt    
       
       outa[motorPin[0]]:= 1
       waitcnt(baseTime + clkfreq/1000000*pulse[0])
       outa[motorPin[0]]:= 0
         
       outa[motorPin[1]]:= 1 
       waitcnt(cnt + clkfreq/1000000*pulse[1])
       outa[motorPin[1]]:= 0
       
       outa[motorPin[2]]:= 1
       waitcnt(cnt + clkfreq/1000000*pulse[2])
       outa[motorPin[2]]:= 0
        
       outa[motorPin[3]]:= 1
       waitcnt(cnt + clkfreq/1000000*pulse[3])
       outa[motorPin[3]]:= 0
       totalElapse := pulse[0] + pulse[1] + pulse[2] + pulse[3]
       waitcnt(baseTime + (clkfreq/1000*20 - clkfreq/1000000*totalElapse))

PRI inspectPulse | i
  i:=0
  repeat while i < 4
    if (pulse[i] < 1100) OR (2000 < pulse[i])
      return 0   ' abnormal pwm
    i++
  return 1


  