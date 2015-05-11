CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000


OBJ

  serial :"ParallaxSerialTerminal.spin"

Var

  long varChar,newValue,type

PUB main | x

  serial.quickStart

  repeat
    if serial.RxCount > 0  
      varChar := serial.CharIn
      serial.str(String("char="))
      serial.decLn(ASCII2Dec(varChar))
      if (48=<varChar AND varChar=<57) 'btw 0-9
        newValue := newValue*10 + ASCII2Dec(varChar)
      elseif(varChar == 77) ' M -> motor
        type := 1  'next 5 digits are (motornumber & pwm)

    else
       if (newValue > 0)
         if(type==1)
           serial.strLn(String("M"))
           type := 0
         serial.str(String("newval="))
         serial.decLn(newValue)
         newValue := 0
         


PRI char2ASCII(charVar)  ' currently not used
  result := byte[charVar]
  ' Don't know how, but this returns ascii code of char

PRI ASCII2Dec(ASCII)
  result := ASCII - 48   