CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
                      
OBJ
  usb            : "Parallax Serial Terminal"
  fNum           : "Float32Full"
  fString        : "FloatString"
  mpu6050        : "MPU-6050acc.spin"
VAR
  'common variables
  byte usbUpdateMode, pidUpdateMode

  'motor variables
  long pulse[4], motorPin[4], motorStack[128]
  byte motorIteration, motorCogId

 'attitude variables
  byte attitudeCogId
  long acc[3], accSquare[3], accNorm, dirCos[3], attitudeStack[128]

  'usb variables
  long newValue, type, usbStack[128]
  byte varChar, motorNumber, usbCogId
  
PUB getReady
  'set calculation base
  fNum.start
  accNorm := fNum.Ffloat(accNorm)
  dirCos[0] := fNum.Ffloat(dirCos[0])
  dirCos[1] := fNum.Ffloat(dirCos[1])
  dirCos[2] := fNum.Ffloat(dirCos[2])

  'usb start
  newUSB

PUB startAutoPilot
  'attitude start
  newAttitude

  'motor start
  newMotor(0,1,2,3)

  'pid start



'===================================================================================================
'===================== MOTOR PART ==================================================================
'===================================================================================================
PRI newMotor(pin0, pin1, pin2, pin3)  {{ constructor }}
  motorPin[0] := pin0  'set pin number for this motor
  motorPin[1] := pin1
  motorPin[2] := pin2
  motorPin[3] := pin3
  waitcnt(cnt + clkfreq)
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

PRI runMotor | baseTime, totalElapse                 {{generating pwm for the motor connected to this pin}}              
  
  initMotor  'physical initialization for this motor 
  motorIteration := 0
  repeat while motorIteration<4
    dira[motorPin[motorIteration]] := 1   'set pin direction for this motor 
    pulse[motorIteration] := 1250         'set default pwm
    motorIteration++
  
  repeat  
    totalElapse:=0
    motorIteration:=0
    'motor iteration
    baseTime := cnt
    repeat while motorIteration<4
      outa[motorPin[motorIteration]]:= 1
      waitcnt(baseTime + clkfreq/1000000*pulse[motorIteration])
      outa[motorPin[motorIteration]]:= 0
      totalElapse := totalElapse + pulse[motorIteration]
      motorIteration++
    waitcnt(baseTime + (clkfreq/1000*25 - clkfreq/1000000*totalElapse))

'===================================================================================================
'===================== PID PART ==================================================================
'===================================================================================================




'===================================================================================================
'===================== COMMUNICATION PART ==================================================================
'===================================================================================================
PRI newUSB
  usb.start(115200)
  startUSB
PRI startUSB
  stopUSB
  usbCogId := cognew(communicate, @usbStack) + 1  'start running motor

PRI stopUSB
  if usbCogId
    cogstop(usbCogId ~ - 1)
      
PRI communicate | i
  repeat
    i:=0 
    if usb.RxCount > 0  
      readCharArray
    else
      i:=0  
      repeat while i=<3
        usb.str(String("M"))
        usb.Dec(i+1)
        usb.Dec(pulse[i])
        i++
      usb.str(String("/"))   
      i:=0
      repeat while i<3
        usb.str(String("C"))
        case i
          0: usb.str(String("x"))
          1: usb.str(String("y"))
          2: usb.str(String("z"))
        usb.str(fstring.FloatToString(dirCos[i]))
        i++
      usb.str(String("/"))  
      i:=0
      
PRI char2ASCII(charVar)  ' currently not used
  result := byte[charVar]
  ' Don't know how, but this returns ascii code of char

PRI ASCII2Dec(ASCII)
  result := ASCII - 48    
  
PRI readCharArray   | newPWM 
   varChar := usb.CharIn
   if (48=<varChar AND varChar=<57) 'btw 0-9
     newValue := newValue*10 + ASCII2Dec(varChar)
   elseif(varChar ==77) ' M -> motor
     type := 1  'next 5 digits are (motornumber & pwm)
   elseif(varChar == 67)
     type := 2  'number is not yet determined

   if (type==1)
     if 11200 < newValue AND newValue < 43000
       motorNumber := newValue/10000
       newPWM := newValue//10000
       case motorNumber
         1: pulse[0] := newPWM
         2: pulse[1] := newPWM  
         3: pulse[2] := newPWM  
         4: pulse[3] := newPWM
       'usb.Dec(motorNumber)
       'usb.str(String("and"))
       'usb.Dec(newPWM)
       'waitcnt(cnt + clkfreq*4) 
       type := 0
       newValue := 0    

'===================================================================================================
'===================== ATTITUDE PART ==================================================================
'===================================================================================================
PRI newAttitude
  startAttitude
PRI startAttitude 
  mpu6050.Start(15,14)   
  stopAttitude
  attitudeCogId := cognew(readAttitude, @attitudeStack) + 1  'start updating current attitude

PRI stopAttitude
  if attitudeCogId
    cogstop(attitudeCogId ~ - 1)

PRI readAttitude | total, i
 ' usb.str(String("im here"))
  repeat
    acc[0] := fNum.Ffloat(mpu6050.GetAx)
    acc[1] := fNum.Ffloat(mpu6050.GetAy)
    acc[2] := fNum.Ffloat(mpu6050.GetAz)
    i := 0
    repeat while i<3
      accSquare[i] := fNum.pow(acc[i], fNum.Ffloat(2)) 
      i++
    total := fNum.Fadd(fNum.Fadd(accSquare[0], accSquare[1]) , accSquare[2])
    accNorm := fNum.FSqr(total)
    i :=0
    repeat while i<3
      dirCos[i] := fNum.fDiv(acc[i], accNorm)
      i++
