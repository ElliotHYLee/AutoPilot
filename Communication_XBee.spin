CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

OBJ
  serial : "FullDuplexSerial"

VAR

  long accPtr[3], gyroPtr[3], eAnglePtr[3]
  long pulsePtr[3]
  long kpPtr, kiPtr, kdPtr, outputPtr, proportionalPtr, derivativePtr, integralPtr

  long systemMode, respondType, respondContent              
  
PUB init 

  result := serial.start(0,1,0,9600)

PUB setPtr(val1, val2, val3, val4, val5, val6, val7, val8, val9, val10, val11)

  accPtr := val1
  gyroPtr := val2
  eAnglePtr := val3

  pulsePtr := val4

  kpPtr := val5
  kiPtr := val6
  kdPtr := val7

  outputPtr := val8
  proportionalPtr := val9
  derivativePtr := val10
  integralPtr := val11
               

PUB communicate(base)
    
    if serial.RxCount > 0  
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

    return base
  


PRI sendTestMsg
  serial.clear   

  serial.str(String("[ax"))
  serial.dec(long[accPtr[0]])
  serial.str(String("]  "))

  serial.str(String("[cx"))
  serial.dec(long[eAnglePtr[0]])
  serial.strLn(String("]"))
  
  serial.str(String("[ay"))
  serial.dec(acc[1])
  serial.str(String("]"))

  serial.str(String("  [cy"))
  serial.dec(eAngle[1])
  serial.strLn(String("]"))  

  serial.str(String("[az"))
  serial.dec(acc[2])
  serial.str(String("]"))  

  serial.str(String("  [cz"))
  serial.dec(eAngle[2])
  serial.strLn(String("]"))  

  waitcnt(cnt + clkfreq/10)        
 
PRI sendOrdinaryMsg | i  

  serial.str(String("[k0"))
  serial.dec(derivative)
  serial.str(String("]"))
  serial.str(String("[k1"))
  serial.dec(proportional)
  serial.str(String("]"))
  serial.str(String("[k3"))
  serial.dec(output)
  serial.str(String("]"))

 'write motor info
  i:=0                 
  repeat while i < 4
    'motor write
    serial.str(String("[m"))
    serial.Dec(i+1)
    serial.Dec(pulse[i])
    serial.str(String("]"))

   'eAngle write
    if i < 3 
      serial.str(String("[c"))
      case i
        0: serial.str(String("x"))
        1: serial.str(String("y"))
        2: serial.str(String("z"))
      serial.dec(eAngle[i])
      serial.str(String("]"))
      {
      serial.str(String("[a"))
      case i
        0: serial.str(String("x"))
        1: serial.str(String("y"))
        2: serial.str(String("z"))
      serial.dec(acc[i])
      serial.str(String("]"))

      serial.str(String("[g"))
      case i
        0: serial.str(String("x"))
        1: serial.str(String("y"))
        2: serial.str(String("z")) 
      serial.dec(gyro[i])
      serial.str(String("]"))
      }
    i++
            

PRI sendPidTestMsg

  serial.clear
  serial.str(String("on/off: "))
  serial.decLn(pidOnOff)  
  serial.str(String("error: "))
  serial.decLn(error)
  serial.str(String("proportional: "))
  serial.decLn(proportional)
  serial.str(String("derivative: "))
  serial.decLn(derivative)
  serial.str(String("integral: "))
  serial.decLn(integral[0])
  serial.str(String("output: "))
  serial.decLn(output)
  waitcnt(cnt + clkfreq/10)
  
                  
PRI respondBack(x)
  case x
    1:
      if respondContent == 1     ' respondContent type 1 = pid gains
        serial.str(String("[pp"))
        serial.dec(kp)
        serial.str(String("]"))
        serial.str(String("[pi"))
        serial.dec(ki)
        serial.str(String("]"))
        serial.str(String("[pd"))
        serial.dec(kd)
        serial.str(String("]"))               
      elseif respondContent ==2    ' respondContent type 1 = pid on/off status 
        serial.str(String("[po"))
        serial.dec(pidOnOff)
        serial.str(String("]"))  

  respondType := 0
  respondContent := 0


PRI char2ASCII(charVar)  ' currently not used
  result := byte[charVar]
  ' Don't know how, but this returns ascii code of char

PRI ASCII2Dec(ASCII)
  result := ASCII - 48    
  
PRI readCharArray   | newPWM, newPidProperty, newRequest, newMode
   varChar := serial.CharIn
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
       serial.RxFlush
       
   elseif (type ==4)    ' systemMode update
     if 10000 < newValue
       newMode := newValue//10000
       systemModeUpdate(newMode)
       serial.RxFlush
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

           