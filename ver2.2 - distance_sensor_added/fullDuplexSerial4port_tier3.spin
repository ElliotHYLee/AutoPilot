CON
_clkmode = xtal1 + pll16x
_xinfreq = 5_000_000

  usb = 0 
  xb = 1


VAR

  long accPtr[3], gyroPtr[3], eAnglePtr[3], refAttPtr[3], newValueCounter
  long pulsePtr[6], throttlePtr

  long xOnPtr, xOffPtr, yOnPtr, yOffPtr, zOnPtr, zOffPtr
  long xKpPtr, xKiPtr, xKdPtr, xOutputPtr, xProPtr, xDerPtr, xIntPtr
  long yKpPtr, yKiPtr, yKdPtr, yOutputPtr, yProPtr, yDerPtr, yIntPtr
  long zKpPtr, zKiPtr, zKdPtr, zOutputPtr, zProPtr, zDerPtr, zIntPtr

  long lcPtr[3] 'local coordinate pointer

  long systemMode, respondType, respondContent              
  long varchar, varchar2, newValue, type,motorNumber,pidUpdateIndex
  long lcAxisNumber, coord
  long pidAxis
  long pidOnOffPtr[3]    
  
OBJ
  com :  "fullDuplexSerial4port_tier2"

PUB main | isReceived, c, localCoordinate[3]

  lcPtr[0] := @localCoordinate[0] 
  lcPtr[1] := @localCoordinate[1]
  lcPtr[2] := @localCoordinate[2]  

  initialize

  communicate
  
 { repeat
   
    isReceived := com.rxIsIn(usb)
    if isReceived
      c := com.charIn(usb)
      com.char(usb, c)
      com.newline(usb)
   }

PUB initialize

  com.initialize
             
PUB communicate | base , x,y

  base := cnt
  repeat
    if com.rxIsIn(usb)  
      readCharArray 
    else
      com.dec(usb, long[lcPtr][0])
      waitcnt(cnt + clkfreq/10)
      

PRI char2ASCII(charVar)  ' currently not used
  result := byte[charVar]
  ' Don't know how, but this returns ascii code of char

PRI ASCII2Dec(ASCII)
  result := ASCII - 48    
  
PRI readCharArray  
   varChar2 := varchar
   varChar := com.CharIn(usb)
   if (48=<varChar AND varChar=<57) 'btw 0-9
     newValue := newValue*10 + ASCII2Dec(varChar)
   elseif(varChar == 108) ' l -> local coordinate
     type := 1  'next 4 digits are (axis number & value)
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
     if 1000 < newValue AND newValue < 7000
       lcAxisNumber := newValue/1000
       coord := newValue//1000
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