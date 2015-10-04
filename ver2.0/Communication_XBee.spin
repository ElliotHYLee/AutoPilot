CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

OBJ
  serial : "ParallaxSerialTerminal.spin"

VAR

  long accPtr[3], gyroPtr[3], eAnglePtr[3]
  long pulsePtr[6], throttlePtr

  long xOnPtr, xOffPtr, yOnPtr, yOffPtr, zOnPtr, zOffPtr
  long xKpPtr, xKiPtr, xKdPtr, xOutputPtr, xProPtr, xDerPtr, xIntPtr
  long yKpPtr, yKiPtr, yKdPtr, yOutputPtr, yProPtr, yDerPtr, yIntPtr
  long zKpPtr, zKiPtr, zKdPtr, zOutputPtr, zProPtr, zDerPtr, zIntPtr

  long systemMode, respondType, respondContent              
  long varchar, varchar2, newValue, type,motorNumber,pidUpdateIndex
  long pidAxis
  long pidOnOffPtr[3]    

PUB main

  init(31, 30, 0, 115200)
  communicate
  
  
PUB init(rx, tx, mode, baud) 

  result := serial.init(rx, tx, mode , baud)
 
PUB setAttPtr(val1, val2, val3)

  accPtr := val1
  gyroPtr := val2
  eAnglePtr := val3

PUB setMotPtr(pwmPtr) | i

  repeat i from 0 to 5
    pulsePtr[i] := pwmPtr[i]
    i++           

PUB setThrottle(valuePtr)

  throttlePtr := valuePtr  
    
PUB setPidOnOffPtr(val)

  pidOnOffPtr[0] := val[0]
  pidOnOffPtr[1] := val[1]
  pidOnOffPtr[2] := val[2]

PUB setXPidPtr(kp, kd, ki, pro, der, int, output)

  xKpPtr  := kp
  xKdPtr  := kd
  xKiPtr  := ki
  xProPtr := pro
  xDerPtr := der
  xIntPtr := int
  xOutputPtr := output

PUB setYPidPtr(kp, kd, ki, pro, der, int, output)

  yKpPtr  := kp
  yKdPtr  := kd
  yKiPtr  := ki
  yProPtr := pro
  yDerPtr := der
  yIntPtr := int
  yOutputPtr := output  

PUB setZPidPtr(kp, kd, ki, pro, der, int, output)

  zKpPtr  := kp
  zKdPtr  := kd
  zKiPtr  := ki
  zProPtr := pro
  zDerPtr := der
  zIntPtr := int
  zOutputPtr := output
  

PUB communicate | base , x,y

  base := cnt
  repeat
    if serial.RxCount > 0  
      readCharArray
    else
      if respondType > 0 ' need to respond to the request from C#
        x := respondType
        y := respondContent
        repeat 2
          respondContent := y
          respondBack(x)
          waitcnt(cnt + clkfreq/10)
        respondType := 0
        respondContent := 0
      else
        if (cnt > base + clkfreq/90)
          'sendPidConst
          sendPidCalc
          sendAttMsg
          sendMotorMsg
          sendThrottleMsg
          base := cnt

PRI sendPidConst

  'p0 = Kp of x axis
  'p1 = Ki of x axis
  'p3 = Kd of x axis
  'p4 = Kp of y axis
  ' ......
  serial.str(String("[p0"))
  serial.dec(long[xKpPtr])
  serial.str(String("]"))
  serial.str(String("[p1"))
  serial.dec(long[xKiPtr])
  serial.str(String("]"))
  serial.str(String("[p2"))
  serial.dec(long[xKdPtr])
  serial.str(String("]"))

  serial.str(String("[p3"))
  serial.dec(long[yKpPtr])
  serial.str(String("]"))
  serial.str(String("[p4"))
  serial.dec(long[yKiPtr])
  serial.str(String("]"))
  serial.str(String("[p5"))
  serial.dec(long[yKdPtr])
  serial.str(String("]"))  

  serial.str(String("[p6"))
  serial.dec(long[zKpPtr])
  serial.str(String("]"))
  serial.str(String("[p7"))
  serial.dec(long[zKiPtr])
  serial.str(String("]"))
  serial.str(String("[p8"))
  serial.dec(long[zKdPtr])
  serial.str(String("]"))

PRI sendPidCalc

  serial.str(String("[k0"))
  serial.dec(long[xProPtr])
  serial.str(String("]"))
  serial.str(String("[k1"))
  serial.dec(long[xDerPtr])
  serial.str(String("]"))
  serial.str(String("[k2"))
  serial.dec(long[xIntPtr])
  serial.str(String("]"))
  serial.str(String("[k3"))
  serial.dec(long[xOutputPtr])
  serial.str(String("]"))

  serial.str(String("[k4"))
  serial.dec(long[yProPtr])
  serial.str(String("]"))
  serial.str(String("[k5"))
  serial.dec(long[yDerPtr])
  serial.str(String("]"))
  serial.str(String("[k6"))
  serial.dec(long[yIntPtr])
  serial.str(String("]"))
  serial.str(String("[k7"))
  serial.dec(long[yOutputPtr])
  serial.str(String("]"))  

  serial.str(String("[k8"))
  serial.dec(long[zProPtr])
  serial.str(String("]"))
  serial.str(String("[k9"))
  serial.dec(long[zDerPtr])
  serial.str(String("]"))
  serial.str(String("[kA"))
  serial.dec(long[zIntPtr])
  serial.str(String("]"))
  serial.str(String("[kB"))
  serial.dec(long[zOutputPtr])
  serial.str(String("]"))


PRI sendThrottleMsg

  serial.str(String("[t"))
  serial.dec(0)
  serial.dec(long[throttlePtr])
  serial.str(String("]"))

PRI sendMotorMsg | i

  repeat i from 0 to 5
    'motor write
    serial.str(String("[m"))
    serial.Dec(i+1)
    serial.Dec(long[pulsePtr][i])
    serial.str(String("]"))      

PRI sendAttMsg | i

  repeat i from 0 to 2
    serial.str(String("[c"))
    case i
      0: serial.str(String("x"))
      1: serial.str(String("y"))
      2: serial.str(String("z"))
    serial.dec(long[eAnglePtr][i])
    serial.str(String("]"))

    

    serial.str(String("[a"))
    case i
      0: serial.str(String("x"))
      1: serial.str(String("y"))
      2: serial.str(String("z"))
    serial.dec(long[accPtr][i])
    serial.str(String("]"))

    serial.str(String("[g"))
    case i
      0: serial.str(String("x"))
      1: serial.str(String("y"))
      2: serial.str(String("z")) 
    serial.dec(long[gyroPtr][i])
    serial.str(String("]"))

                
PRI respondBack(x)
  case x
    1:
      if respondContent == 1     ' respondContent type 1 = pid gains
        sendPidConst                      
      elseif respondContent == 2    ' respondContent type 1 = pid on/off status 
        sendPidOnOffStatus
                
  respondType := 0
  respondContent := 0

PRI sendPidOnOffStatus

  serial.str(String("[o0"))
  serial.dec(long[pidOnOffPtr][0])
  serial.str(String("]"))  
  serial.str(String("[o1"))
  serial.dec(long[pidOnOffPtr][1])
  serial.str(String("]"))
  serial.str(String("[o2"))
  serial.dec(long[pidOnOffPtr][2])
  serial.str(String("]"))
        
PRI char2ASCII(charVar)  ' currently not used
  result := byte[charVar]
  ' Don't know how, but this returns ascii code of char

PRI ASCII2Dec(ASCII)
  result := ASCII - 48    
  
PRI readCharArray   | newPWM, newPidProperty, newRequest, newMode
   varChar2 := varchar
   varChar := serial.CharIn
   if (48=<varChar AND varChar=<57) 'btw 0-9
     newValue := newValue*10 + ASCII2Dec(varChar)
   elseif(varChar == 77) ' M -> motor
     type := 1  'next 5 digits are (motornumber & pwm)
   elseif(varChar == 80) ' P -> PID constant change
     type := 2  'next 5 digits are (PID type and info)
   elseif(varChar == 82) ' R -> request of specific info
     type := 3  ' next 2 digits are request types
   elseif(varChar == 68) ' D -> system mode update
     type := 4  ' next 5 digits are mode types
   elseif(varChar == 79) ' O -> PID on/off
     type := 5  ' next 5 digits are mode types
   elseif(varChar2 ==  84 AND varchar == 72) ' TH -> throttle
     type := 6  
                            
   if (type==1)
     if 11099 < newValue AND newValue < 63000
       motorNumber := newValue/10000
       newPWM := newValue//10000
       case motorNumber
         1: long[pulsePtr][0] := newPWM
         2: long[pulsePtr][1] := newPWM  
         3: long[pulsePtr][2] := newPWM  
         4: long[pulsePtr][3] := newPWM
         5: long[pulsePtr][4] := newPWM
         6: long[pulsePtr][5] := newPWM
       type := 0
       newValue := 0
   elseif (type == 2)   ' PID constant update
     if 9_999_999 < newValue
       pidUpdateIndex := newValue/10_000_000
       newPidProperty := newValue//10_000_000
       case pidUpdateIndex
         1: long[xKpPtr] := newPidProperty
         2: long[xKiPtr] := newPidProperty
         3: long[xKdPtr] := newPidProperty
         4: long[yKpPtr] := newPidProperty
         5: long[yKiPtr] := newPidProperty
         6: long[yKdPtr] := newPidProperty
         7: long[zKpPtr] := newPidProperty
         8: long[zKiPtr] := newPidProperty
         9: long[zKdPtr] := newPidProperty                  
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
       case respondType
         1: respondContent := newRequest
       respondContent := newRequest   ' respond content 2 = pid on/off
       respondBack(respondType)
       if (newValue == 12)
         sendPidOnOffStatus   
       'waitcnt(cnt + clkfreq*2)
       type := 0
       newValue := 0
       'serial.RxFlush
       
   elseif (type == 4)    ' systemMode update
     if 10000 < newValue
       newMode := newValue//10000
       systemModeUpdate(newMode)
       serial.RxFlush
       newValue := 0
       respondContent := 2 'respond content 2 = pid on/off
       respondBack(1)      'repond type 1 = all pid types

   elseif (type == 5)   ' pid on/off status
     if 9 < newValue  AND newValue < 50  'pif on off msg: o10 -> x axis off
       pidAxis := newValue/10
       setPidStatus(newValue)
      ' serial.str(String("pid on off request :"))
      ' serial.decLn(newValue)
       'waitcnt(cnt + clkfreq*2)
       type := 0
       newValue := 0
       sendPidOnOffStatus

   elseif (type == 6)   ' Throttle value
     if 1099 < newValue AND newValue < 2500
       updateThrottle(newValue)
       serial.str(String("throttle request :"))
       serial.decLn(newValue)
       type := 0
       newValue := 0
       
PRI updateThrottle(val)| i

  long[throttlePtr] := val
  repeat i from 0 to 5
    long[pulsePtr][i] := val 
  
PRI systemModeUpdate(mode)

  systemMode := mode
  case mode
     1: 'idle
       pidOff
       updateThrottle(1100)
       
     2: 'prepare
       pidOn
       updateThrottle(1200)
       
     3: 'hover
       pidOn
     4: 'navigation
       pidOn

PRI setPidStatus(val)

  if val== 10
    pidOffX
  if val == 11
    pidOnX
  if val == 20
    pidOffY
  if val == 21
    pidOnY
  if val == 30
    pidOffZ
  if val == 31
    pidOnZ


PRI pidOn
  pidOnX
  pidOnY
  pidOnZ
  
PRI pidOff
  pidOffX
  pidOffY
  pidOffZ
  
PRI pidOnX
  long[pidOnOffPtr][0] := 1
  
PRI pidOnY
  long[pidOnOffPtr][1] := 1
   ' serial.str(String("pidY is on"))
  'waitcnt(cnt + clkfreq)
  
PRI pidOnZ
  long[pidOnOffPtr][2] := 1
  
PRI pidOffX
  long[pidOnOffPtr][0] := 0
  
PRI pidOffY
  long[pidOnOffPtr][1] := 0

PRI pidOffZ
  long[pidOnOffPtr][2] := 0

  