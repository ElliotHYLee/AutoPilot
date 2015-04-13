CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000



  
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
  usb.dec(derivative)
  usb.str(String("]"))
  usb.str(String("[k1"))
  usb.dec(proportional)
  usb.str(String("]"))
  usb.str(String("[k3"))
  usb.dec(output)
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
            

PRI sendPidTestMsg

  usb.clear
  usb.str(String("on/off: "))
  usb.decLn(pidOnOff)  
  usb.str(String("error: "))
  usb.decLn(error)
  usb.str(String("proportional: "))
  usb.decLn(proportional)
  usb.str(String("derivative: "))
  usb.decLn(derivative)
  usb.str(String("integral: "))
  usb.decLn(integral[0])
  usb.str(String("output: "))
  usb.decLn(output)
  waitcnt(cnt + clkfreq/10)
  
                  
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

           