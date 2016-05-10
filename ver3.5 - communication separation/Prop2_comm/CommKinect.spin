CON
_clkmode = xtal1 + pll16x
_xinfreq = 5_000_000

VAR

  long com_listener_CogId
  long lcPtr[3] 'local coordinate pointer
  long varChar, varChar2, newValue, type
  long lcAxisNumber, coord
OBJ
  com :  "ParallaxSerialTerminal.Spin"

PUB main | x

  x:=0
  initialize

  'communicate

  repeat
    if (com.RxCount > 0)
      com.clear  
      x := com.charIn
      com.char(x)
       

PUB initialize

  com_listener_CogId := com.quickStart

PUB setLocalCoordinate(valuePtr)

  lcPtr[0] := valuePtr[0]  
  lcPtr[1] := valuePtr[1]
  lcPtr[2] := valuePtr[2]  

  
'=================================
' Main Loop
'=================================         
PUB communicate

  repeat
    if com.RxCount  
      readCharArray_usb
   

'=============================================
'USB communication routine
'============================================
PRI readCharArray_usb  
   varChar2 := varchar
   varChar := com.CharIn
   if (48=<varChar AND varChar=<57) 'btw 0-9
     newValue := newValue*10 + ASCII2Dec(varChar)
   elseif(varChar == 108) ' l -> local coordinate
     type := 1  'next 5 digits are (axis number & value)
     newValue := 0
   {
   axis number list:
   1 : positive x
   2 : negative x
   3 : positive y
   4 : negative y
   5 : positive z
   6 : negative z
   }
                        
   if (type==1)
     if 10000 < newValue AND newValue < 70000
       lcAxisNumber := newValue/10000
       coord := newValue//10000
       case lcAxisNumber
         1: long[lcPtr][0] := coord
         2: long[lcPtr][0] := -coord  
         3: long[lcPtr][1] := coord  
         4: long[lcPtr][1] := -coord
         5: long[lcPtr][2] := coord
         6: long[lcPtr][2] := -coord
       type := 0
       newValue := 0
       coord := 0
        
'=================================
' Auxiliary Loop
'================================       

PRI char2ASCII(charVar)  ' currently not used
  result := byte[charVar]
  ' Don't know how, but this returns ascii code of char

PRI ASCII2Dec(ASCII)
  result := ASCII - 48    

{
PUB sendCtrlRef

  com.str(string("[sx"))           
  com.dec(long[refAttPtr][0])
  com.str(string("]"))

  com.str(string("[sy"))           
  com.dec(xb, long[refAttPtr][1])
  com.str(xb, string("]"))

  com.str(xb, string("[sz"))           
  com.dec(xb, long[refAttPtr][2])
  com.str(xb, string("]"))  

 
PUB sendLocalCoordinate(port) 


  com.str(port, string("[lx"))
  com.dec(port, long[lcPtr][0])
  com.str(port, string("]"))

  com.str(port, string("[ly"))
  com.dec(port, long[lcPtr][1])
  com.str(port, string("]"))

  com.str(port, string("[lz"))
  com.dec(port, long[lcPtr][2])
  com.str(port, string("]"))

 }









       