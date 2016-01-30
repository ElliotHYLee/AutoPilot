CON
_clkmode = xtal1 + pll16x
_xinfreq = 5_000_000

  usb = 0 
  xb = 1


VAR

  byte com_listener_CogId

  long accPtr[3], gyroPtr[3], eAnglePtr[3], refAttPtr[3], newValueCounter
  long pulsePtr[6], throttlePtr, dist_ground_ptr

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

  com_listener_CogId := com.initialize

PUB setDistPtr(val)

  dist_ground_ptr := val
 
PUB setAttPtr(val1, val2, val3)

  accPtr := val1
  gyroPtr := val2
  eAnglePtr := val3

PUB setMotPtr(pwmPtr) | i

  repeat i from 0 to 5
    pulsePtr[i] := pwmPtr[i]
    i++           

PUB setTargetAttitude(valuePtr)

  refAttPtr[0] := valuePtr[0]
  refAttPtr[1] := valuePtr[1]
  refAttPtr[2] := valuePtr[2]
  newValueCounter :=0 ' counts digits of newValue from GCS

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
  
'=================================
' Main Loop
'=================================         
PUB communicate | base , x,y

  base := cnt
  repeat
    ' communication with on board computer
    if com.rxIsIn(usb)  
      readCharArray_usb
    else
      com.dec(usb, long[lcPtr][0])
      waitcnt(cnt + clkfreq/10)


    ' communication with GCS
    if com.rxIsIn(xb)
      readCharArray_xb
    else
      if (cnt > base + clkfreq/90)
          sendPidConst
          sendPidCalc
          sendAttMsg
          sendMotorMsg
          sendThrottleMsg
          sendDistGrdMsg

          base := cnt 
'=================================
' Auxiliary Loop
'================================       

PRI char2ASCII(charVar)  ' currently not used
  result := byte[charVar]
  ' Don't know how, but this returns ascii code of char

PRI ASCII2Dec(ASCII)
  result := ASCII - 48    

'=============================================
'USB communication routine
'============================================
PRI readCharArray_usb  
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

'=============================================
'Xbee communication routine
'============================================
PRI readCharArray_xb
   varChar2 := varchar
   varChar := com.CharIn(xb)
   if (48=<varChar AND varChar=<57) 'btw 0-9
     newValue := newValue*10 + ASCII2Dec(varChar)
   elseif(varChar == 108) ' l -> local coordinate
     type := 1  'next 4 digits are (axis number & value)


PRI sendPidConst

  'p0 = Kp of x axis
  'p1 = Ki of x axis
  'p3 = Kd of x axis
  'p4 = Kp of y axis
  ' ......
  com.str(xb, String("[p0"))
  com.dec(xb, long[xKpPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[p1"))
  com.dec(xb, long[xKiPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[p2"))
  com.dec(xb, long[xKdPtr])
  com.str(xb, String("]"))

  com.str(xb, String("[p3"))
  com.dec(xb, long[yKpPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[p4"))
  com.dec(xb, long[yKiPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[p5"))
  com.dec(xb, long[yKdPtr])
  com.str(xb, String("]"))  

  com.str(xb, String("[p6"))
  com.dec(xb, long[zKpPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[p7"))
  com.dec(xb, long[zKiPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[p8"))
  com.dec(xb, long[zKdPtr])
  com.str(xb, String("]"))

PRI sendPidCalc

  com.str(xb, String("[k0"))
  com.dec(xb, long[xProPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[k1"))
  com.dec(xb, long[xDerPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[k2"))
  com.dec(xb, long[xIntPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[k3"))
  com.dec(xb, long[xOutputPtr])
  com.str(xb, String("]"))

  com.str(xb, String("[k4"))
  com.dec(xb, long[yProPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[k5"))
  com.dec(xb, long[yDerPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[k6"))
  com.dec(xb, long[yIntPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[k7"))
  com.dec(xb, long[yOutputPtr])
  com.str(xb, String("]"))  

  com.str(xb, String("[k8"))
  com.dec(xb, long[zProPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[k9"))
  com.dec(xb, long[zDerPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[kA"))
  com.dec(xb, long[zIntPtr])
  com.str(xb, String("]"))
  com.str(xb, String("[kB"))
  com.dec(xb, long[zOutputPtr])
  com.str(xb, String("]"))


PRI sendThrottleMsg

  com.str(xb, String("[t"))
  com.dec(xb, 0)
  com.dec(xb, long[throttlePtr])
  com.str(xb, String("]"))

PRI sendMotorMsg | i

  repeat i from 0 to 5
    'motor write
    com.str(xb, String("[m"))
    com.Dec(xb, i+1)
    com.Dec(xb, long[pulsePtr][i])
    com.str(xb, String("]"))

PRI sendDistGrdMsg

  com.str(xb, String("[dg"))
  com.dec(xb, long[dist_ground_ptr])
  com.str(xb, String("]"))         

PRI sendAttMsg | i

  repeat i from 0 to 2
    com.str(xb, String("[c"))
    case i
      0: com.str(xb, String("x"))
      1: com.str(xb, String("y"))
      2: com.str(xb, String("z"))
    com.dec(xb, long[eAnglePtr][i])
    com.str(xb, String("]"))

    com.str(xb, String("[a"))
    case i
      0: com.str(xb, String("x"))
      1: com.str(xb, String("y"))
      2: com.str(xb, String("z"))
    com.dec(xb, long[refAttPtr][i])
    com.str(xb, String("]"))
    {
    serial.str(String("[g"))
    case i
      0: serial.str(String("x"))
      1: serial.str(String("y"))
      2: serial.str(String("z")) 
    serial.dec(long[gyroPtr][i])
    serial.str(String("]"))
    }
                
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

  com.str(xb, String("[o0"))
  com.dec(xb, long[pidOnOffPtr][0])
  com.str(xb, String("]"))  
  com.str(xb, String("[o1"))
  com.dec(xb, long[pidOnOffPtr][1])
  com.str(xb, String("]"))
  com.str(xb, String("[o2"))
  com.dec(xb, long[pidOnOffPtr][2])
  com.str(xb, String("]"))
        














       