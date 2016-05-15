CON
_clkmode = xtal1 + pll16x
_xinfreq = 5_000_000

  usb = 0 
  xb = 1


VAR

  long com_listener_CogId

  long magPtr[3], accPtr[3], gyroPtr[3], eAnglePtr[3], refAttPtr[3], newValueCounter
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
  long pidOnOffPtr[3], navPidOnOffPtr[3] 
  
OBJ
  com :  "fullDuplexSerial4port_tier2"

PUB main | isReceived, c, localCoordinate[3]

  lcPtr[0] := @localCoordinate[0] 
  lcPtr[1] := @localCoordinate[1]
  lcPtr[2] := @localCoordinate[2]  

  initialize

  'communicate

  repeat
    if com.rxIsIn(xb)
      c := com.charIn(xb)
      com.char(xb, c)
      com.newline(xb)

PUB initialize

  com_listener_CogId := com.initialize

PUB setLocalCoordinate(valuePtr)

  lcPtr[0] := valuePtr[0]  
  lcPtr[1] := valuePtr[1]
  lcPtr[2] := valuePtr[2]  

PUB setDistPtr(val)

  dist_ground_ptr := val
 
PUB setAttPtr(val1, val2, val3, val4)

  accPtr := val1
  gyroPtr := val2
  eAnglePtr := val3
  magPtr := val4

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


PUB setNavPidOnOffPtr(val)

  navPidOnOffPtr[0] := val[0]
  navPidOnOffPtr[1] := val[1]
  navPidOnOffPtr[2] := val[2]  

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
PUB communicate | base , counter, isUsbTurn

  counter := 0
  isUsbTurn := -1
  base := cnt 
  repeat

    if isUsbTurn AND com.rxIsIn(usb)
      'com.RxFlush(xb)
      
      ' communication with on board computer
      'if com.rxIsIn(usb)   
      readCharArray_usb

      if (counter>100)
          isUsbTurn := 0
          counter := 0
      counter++

    else
       'com.RxFlush(usb)

      ' communication with GCS (Xbee)
      if com.rxIsIn(xb)
        readCharArray_xb
      if (counter>100)
          isUsbTurn := -1
          counter := 0
      else
        if (cnt > base + clkfreq/10)
          sendPidConst
          sendPidCalc
          sendMagMsg
          sendAttMsg
          sendMotorMsg
          sendThrottleMsg
          sendDistGrdMsg
          sendLocalCoordinate(xb)
          sendCtrlRef
           
          base := cnt 

      counter++




       
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

PUB sendCtrlRef

  com.str(xb, string("[sx"))           
  com.dec(xb, long[refAttPtr][0])
  com.str(xb, string("]"))

  com.str(xb, string("[sy"))           
  com.dec(xb, long[refAttPtr][1])
  com.str(xb, string("]"))

  com.str(xb, string("[sz"))           
  com.dec(xb, long[refAttPtr][2])
  com.str(xb, string("]"))  




PUB sendMagMsg

  com.str(xb, string("[qx"))           
  com.dec(xb, long[magPtr][0])
  com.str(xb, string("]"))

  com.str(xb, string("[qy"))           
  com.dec(xb, long[magPtr][1])
  com.str(xb, string("]"))

  com.str(xb, string("[qz"))           
  com.dec(xb, long[magPtr][2])
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




'=============================================
'Xbee communication routine
'============================================
PRI readCharArray_xb  | newPWM, newPidProperty, newRequest, newMode
   varChar2 := varchar
   varChar := com.CharIn(xb)
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
   elseif(varChar2 ==  82 AND varchar == 65) ' RA -> Reference AttitudeR
     type := 7      

    
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
       com.RxFlush(xb)
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
       'serial.str(String("throttle request :"))
       'serial.decLn(newValue)
       type := 0
       newValue := 0
       
   elseif (type == 7)   ' Reference attitude
     'newValueCounter++
     'if 5 < newValueCounter
        if 100000 =< newValue AND newValue =< 618000
          'x axis: -90 deg = 19000,+90 deg = 9000
          'y axis: -90 deg = 39000,+90 deg = 29000
          'z axis: -90 deg = 59000,+90 deg = 49000
          
          
          updateRefAtt(newValue)
          
          type := 0
          newValue := 0


PRI updateRefAtt(x) | axis, targetAtt

  axis := x/100_000 
  targetAtt := x//100_000
          

  case axis
        1: long[refAttPtr][0] := targetAtt
        2: long[refAttPtr][0] := -targetAtt
        3: long[refAttPtr][1] := targetAtt
        4: long[refAttPtr][1] := -targetAtt
        5: long[refAttPtr][2] := targetAtt
        6: long[refAttPtr][2] := -targetAtt

  {
  if (  0 <= pnAxis AND  pnAxis <= 3)
    if ( newTargetValue < 601)
      case pnAxis
        0: long[refAttPtr][0] := newTargetValue
        1: long[refAttPtr][0] := -newTargetValue
        2: long[refAttPtr][1] := newTargetValue
        3: long[refAttPtr][1] := -newTargetValue
  else
    case pnAxis
      4: long[refAttPtr][2] := newTargetValue
      5: long[refAttPtr][2] := -newTargetValue
    }
 
PRI updateThrottle(val)| i

  long[throttlePtr] := val
  repeat i from 0 to 5
    long[pulsePtr][i] := val 

          
PRI systemModeUpdate(mode)

  systemMode := mode
  case mode
     1: 'idle
       updateThrottle(1100)
       pidOff
       navPidOff
       
     2: 'prepare
       pidOn
       updateThrottle(1200)

     3: 'pre take off routine
       updateThrottle(1300)
       sendMotorMsg
       waitcnt(cnt + clkfreq/1)
       updateThrottle(1400)
       sendMotorMsg
       waitcnt(cnt + clkfreq/1)
       updateThrottle(1450)
       sendMotorMsg
       waitcnt(cnt + clkfreq/1)       
       
     4: 'take-off and hover
       'pidOn
       navPidOn

     5: 'navigation
       'pidOn

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

'===========
PRI navPidOn
  navPidOnX
  navPidOnY
  navPidOnZ
  
PRI navPidOff
  navPidOffX
  navPidOffY
  navPidOffZ
  
PRI navPidOnX
  long[navPidOnOffPtr][0] := 1
  
PRI navPidOnY
  long[navPidOnOffPtr][1] := 1

PRI navPidOnZ
  long[navPidOnOffPtr][2] := 1
  
PRI navPidOffX
  long[navPidOnOffPtr][0] := 0
  
PRI navPidOffY
  long[navPidOnOffPtr][1] := 0

PRI navPidOffZ
  long[navPidOnOffPtr][2] := 0



'=====

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
    com.dec(xb, long[accPtr][i])
    com.str(xb, String("]"))
    
    com.str(xb, String("[g"))
    case i
      0: com.str(xb,String("x"))
      1: com.str(xb,String("y"))
      2: com.str(xb,String("z")) 
    com.dec(xb,long[gyroPtr][i])
    com.str(xb,String("]"))
    
                
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
        














       