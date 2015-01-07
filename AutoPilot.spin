CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
                      
OBJ
  usb            : "Parallax Serial Terminal"
  fNum           : "FloatMath.spin"
  mpu6050        : "sensor.spin"
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
  long eAngle10E5[3], attitudeStack[64] , attitudeCogId , sensorCodId
  
  'usb variables
  long newValue, type, usbStack[64],usbCogId, pstCodId
  long varChar, motorNumber 

  'pid variables
  long pidStack[128], pidCogId
  long targetEAnlge10E5[3]
  long kp, ki, kd, pidUpdateIndex, prevTime[2] , fErrorPrev[2]
  byte pidOnOff

PUB startAutoPilot

  'usb start
  newUSB
  
 'attitude start
  newAttitude

  'motor start
  newMotor(0,1,2,3)

  waitcnt(cnt + clkfreq*3)
  'pid start
  startPID

  cogstop(0)

'===================================================================================================
'===================== PID PART ==================================================================
'===================================================================================================
PRI pidOn
  pidOnOff := 1
  
PRI pidOff
  pidOnOff := 0

PRI stopPID
  if pidCogId
    cogstop(pidCogId ~ - 1)

PRI startPID
  stopPID
  pidOn
  pidCogId := cognew(runPID, @pidStack) + 1  'start running pid controller

PRI runPID 
  kp := 15
  ki := 0
  kd := 150
  repeat
    if pidOnOff == 1
      pidAxis(1,3) ' x axis pid set ( red arms of the drone)
      pidAxis(0,2) ' y axis pid set ( white arms of the drone)  
    else
      'Do nothing  

PRI pidAxis(oneMoter, anotherMoter) | currentTime, fError, fDeltaError, fError10E5, fTenE8, fkp, fki, fkd, fdt, fProportional, fIntegral, fDerivative, fOutPut, outPut, dt, axis

  if oneMoter == 1         'for x axis 
    axis := 0
  else                     'for y axis 
    axis := 1

  '============================
  '=== float calculation region
  '============================
  'just a constant; last three 000 is for PID constant so that, for ex,  kp can vary 1 ~ 9000
  fTenE8 := fNum.FFloat(100_000_000) 

  ' prepare constants; kp, ki, kd are global variables in integer
  fkp := fNum.FFloat(kp)
  fki := fNum.FFloat(ki) 
  fkd := fNum.FFloat(kd)
  
  fError10E5 := fNum.FFloat(targetEAnlge10E5[axis]- eAngle10E5[axis]) 'eAngle10E5/(10^5) = eAngle_Raw itself varies from 0 to 8190
  ' here, if drone tilted to x-axis's left, RawCx <0 then, error  > 0
  
  fError :=  fNum.FDiv(fError10E5, fTenE8) ' now fError is from(-8.1,  8.1)  <- this makes sense because could increase by 1 for pulse

  '---------------------------------------------------------------
  '------------------- consider this part again.., might not need it.
  ' get time difference
  currentTime := cnt                                                     
  dt := currentTime - prevTime[axis]
  fdt := fNum.FDiv(fNum.FFloat(dt), fNum.FFloat(clkfreq))    'dt is in second; dt is about 0.008  or 0.002 second
  fdt := fNum.FMul(fdt, fNum.FFloat(1000)) 'now dt in milli second: about 8 (ms) for both axii, but about 2 (ms) if one axis

  fDeltaError := fNum.FSub(fError, fErrorPrev[axis]) ' <- this will naturally include the division; (detalError / dt) -> a certain "dt" because of the loop
  'fDeltaError := fNum.FDiv(fDeltaError, fdt) ' is this part necessary??
  '-------------------------------------------------------------------
  '-------------------------------------------------------------------
  if (axis == 0)
    'usb.dec(111)
    'usb.newline
    usb.str(fStr.FloatToString(fdt))
    usb.newline
  
  fProportional := fNum.FMul(fkp, fError)
  fIntegral := fNum.FMul(ki, fNum.FAdd(fErrorPrev,fError))
  'fDerivative := fNum.FMul(kd, fNum.FDiv(fDeltaError,fdt))
  fDerivative := fNum.FMul(kd, fDeltaError) ' I guess dError may be (0,1)

  fOutPut := fNum.FAdd(fProportional, fNum.FAdd(fIntegral, fDerivative))

  ' set variables (global var) for next iteration
  fErrorPrev[axis] := fError
  prevTime[axis] := currentTime

  '========================
  '=== back to motor output
  '========================
  outPut := fNum.FRound(fOutPut)
    
  if fError > 0  ' when tilted to positive x axis - increase motor 4 , or 3 for positive y axis
    if pulse[anotherMoter] + outPut  =< 1350
      pulse[anotherMoter] := pulse[anotherMoter] + outPut
    if (pulse[oneMoter] - outPut) => 1200
      pulse[oneMoter] := pulse[oneMoter] - outPut
  elseif fError < 0  ' when tilted to negative x axis - increase motor 2, or 1 negative y axis
    if pulse[oneMoter] + (-outPut) =< 1350
      pulse[oneMoter] := pulse[oneMoter] + (-outPut) 
      if (pulse[anotherMoter] - (-outPut)) => 1200
       pulse[anotherMoter] := pulse[anotherMoter] - (-outPut)   


'===================================================================================================
'===================== MOTOR PART ==================================================================
'===================================================================================================
PRI newMotor(pin0, pin1, pin2, pin3)  {{ constructor }}
  motorPin[0] := pin0  'set pin number for this motor
  motorPin[1] := pin1
  motorPin[2] := pin2
  motorPin[3] := pin3
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

'===================================================================================================
'===================== COMMUNICATION PART ==================================================================
'===================================================================================================
PRI newUSB
  pstCodId:=usb.start(115200)
  'startUSB
  
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
      if respondContent == 1
        usb.str(String("[pp"))
        usb.dec(kp)
        usb.str(String("]"))
        usb.str(String("[pi"))
        usb.dec(ki)
        usb.str(String("]"))
        usb.str(String("[pd"))
        usb.dec(kd)
        usb.str(String("]"))               
      elseif respondContent ==2
        usb.str(String("[po"))
        usb.dec(pidOnOff)
        usb.str(String("]"))  

  respondType := 0
  respondContent := 0
 
PRI sendOrdinaryMsg | i  
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
           eAngle10E5[i] := mpu6050.GetCx  
        1: usb.str(String("y"))
           eAngle10E5[i] := mpu6050.GetCy  
        2: usb.str(String("z"))
           eAngle10E5[i] := mpu6050.GetCz 
      usb.dec(eAngle10E5[i])
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
     if 9999 < newValue
       pidUpdateIndex := newValue/10000
       newPidProperty := newValue//10000
       'waitcnt(cnt + clkfreq*5)
       case pidUpdateIndex
         1: pidOnOff := newPidProperty
         2: kp := newPidProperty
         3: ki := newPidProperty
         4: kd := newPidProperty
       type := 0
       newValue := 0
       respondContent := 1 ' respond content 1 = pid constants
       respondBack(1) 'repond type 1 = all pid types

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
'===================== ATTITUDE PART ==================================================================
'===================================================================================================
PRI newAttitude
  startAttitude
  
PRI startAttitude 
  sensorCodId:=mpu6050.Start(15,14, 98) ' scl, sda, cFilter portion in %   