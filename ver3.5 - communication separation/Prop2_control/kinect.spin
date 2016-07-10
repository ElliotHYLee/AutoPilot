CON
        
  _clkmode = xtal1 + pll16x     
  _xinfreq = 6_250_000

  db = 0
  kin = 1

OBJ
  kinect        : "fullDuplexSerial4port_tier2.spin"
  
VAR


  byte kinByte

  long lcPtr[3], lplc[3], tempLc[3], lpcon[3]
  

PUB main


PUB init

  kinect.quickStartKinect
  lpcon[0] := 80
  lpcon[1] := 50
  lpcon[2] := 50

PUB setLocalCoord(val)

  lcPtr := val

PUB runKinect | i 

  repeat
    if kinect.rxIsIn(kin)
      readCharArray_kin
     

PUB readCharArray_kin        

  case consume
    "A" : getX
    "B" : getY
    "C" : getZ  

PUB getX 
  
  tempLc[0] := 800 #> getDec <#3000
  lplc[0] := lpcon[0]*templc[0]/100 + (100-lpcon[0])*lplc[0]/100
  long[lcPtr][0] := lplc[0]
  
PUB getY 
  tempLc[1] := -800 #> getDec <#800
  lplc[1] := lpcon[1]*templc[1]/100 + (100-lpcon[1])*lplc[1]/100     
  long[lcPtr][1] := lplc[1]

PUB getZ 
  tempLc[2] := -800 #> getDec <#800 
  lplc[2] := lpcon[2]*templc[2]/100 + (100-lpcon[0])*lplc[2]/100     
  long[lcPtr][2] := lplc[2]
  
PUB consume  | tick
  result := kinByte
  kinByte := kinect.CharIn(kin)

  tick := cnt
  repeat while(cnt =< tick + clkfreq/1000)
  'It needs to pause for a while in case of xbee is slow and next buffer can be temporarily empty
  ' , which doesn't mean the end of data


PRI getDec | value, sign, answer

  value := 0
  if (kinByte == "-")
    sign := -1
    consume
  elseif ((kinByte => "0") and (kinByte =< "9"))
    sign := 1
    value := consume - "0"

  repeat 
      if ((kinByte => "0") and (kinByte =< "9"))
        value *= 10
        value += consume - "0"
      else
        quit
  answer := sign*value
  return answer

  