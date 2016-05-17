CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

VAR

long xKpPtr, xKdPtr, xKiPtr, xErr, xPro, xDer, xInt, xIntInt, xOutput
long yKpPtr, yKdPtr, yKiPtr, yErr, yPro, yDer, yInt, yIntInt, yOutput
long zKpPtr, zKdPtr, zKiPtr, zErr, zPro, zDer, zInt, zIntInt, zOutput

long eAngle[3], gyro[3]

long oldDer_roll,oldDer_pitch, oldDer_yaw

OBJ

  math   : "MyMath.spin"
  
PUB getErrX
  return xErr
PUB getProX
  return xPro
PUB getDerX
  return xDer
PUB getIntX
  return xInt

PUB getErrY
  return yErr
PUB getProY
  return yPro
PUB getDerY
  return yDer
PUB getIntY
  return yInt

PUB getErrZ
  return zErr
PUB getProZ
  return zPro
PUB getDerZ
  return zDer
PUB getIntZ
  return zInt
           
PUB setAttVal(eAnglePtr, gyroPtr)

 eAngle := eAnglePtr
 gyro := gyroPtr
 
PUB setXaxis(kpPtr, kdPtr, kiPtr)

  xKpPtr := kpPtr
  xKdPtr := kdPtr
  xKiPtr := kiPtr

PUB setYaxis(kpPtr, kdPtr, kiPtr)

  yKpPtr := kpPtr
  yKdPtr := kdPtr
  yKiPtr := kiPtr

PUB setZaxis(kpPtr, kdPtr, kiPtr)

  zKpPtr := kpPtr
  zKdPtr := kdPtr
  zKiPtr := kiPtr

PUB calcPIDRoll(targetVal): output| alpha, angVel ' controlling motor pulse 0 1 2

  
  xErr := (targetVal- long[eAngle][1])  'eAngle[1] = roll
 ' if (-100 > xErr) OR (xErr>100)
  xPro :=  (xErr * long[xKpPtr])/10000

  angVel := long[gyro][0]/131   '  deg/s
  alpha := 1000'long[xKdPtr]
  oldDer_roll := (angVel*alpha )/1000 + (oldDer_roll*(1000 - alpha))/1000

  xDer := oldDer_roll*long[xKdPtr]/10000    '(angVel*long[xKdPtr]/1000)
  xIntInt := (xIntInt + (xErr*long[xKiPtr])/100_000)
  xInt := -50 #> (xIntInt)/1000  <# 50   
     
  if -5000 < xErr AND xErr < 5000 
    output := (xPro - xDer + xInt)
  else
    output :=  xPro - xDer
    xInt := 0

PUB calcPIDPitch(targetVal): output| alpha, angVel  ' controlling motor pulse 0 and 5  - pitch control

  yErr := (targetVal- long[eAngle][0]) 'eAngle[0] = pitch
  yPro := (yErr * long[yKpPtr] )/10000    

  angVel := long[gyro][1]/131  
  alpha := 1000'long[yKdPtr]
  oldDer_pitch := (angVel*alpha )/1000 + (oldDer_pitch*(1000 - alpha))/1000
  yDer := oldDer_pitch*long[yKdPtr]/10000

  yIntInt := (yIntInt + (yErr*long[yKiPtr])/100_000)
         
  yInt := -50 #> (yIntInt)/1000  <# 50   
  if -5000 < yErr AND yErr < 5000 
    output := yPro - yDer + yInt
  else
    output :=  yPro - yDer
    yInt := 0

PUB calcPIDYaw(targetVal): output   | alpha, angVel, angLim, zIntLim

  angLim := 1000    ' 10 deg
  zIntLim := 20
  zErr := (targetVal- long[eAngle][2])  'positive err = need to rotate cw -> up ccw motors

  if (zErr>18000)
    zErr := zErr - 36000
  elseif(zErr < -18000)
    zErr := zErr + 36000

  
  if (zErr > angLim) 'if Error is greater than 10 deg, need to rotate CW
    zErr := angLim
    'long[zKdPtr] /=10
    'long[zKiPtr] :=0
  elseif (zErr < -angLim)
    zErr := -angLim
    'long[zKdPtr] /=10 
    'long[zKiPtr] :=0  
  
   
  zPro := (zErr * long[zKpPtr])/10000
    'zDer := (long[gyro][2]/131 * long[zKdPtr] )/100
     
  angVel := long[gyro][2]/131  
  alpha := 1000'long[zKdPtr]
     
  oldDer_yaw := (angVel*alpha )/1000 + (oldDer_yaw*(1000 - alpha))/1000
  zDer := oldDer_yaw *long[zKdPtr]/1000
     
  zIntInt := (zIntInt + (zErr*long[zKiPtr])/100_000)

  if (zIntInt > zIntLim)
    zInt := zIntLim
  elseif(zIntInt < -zIntLim)
    zInt := -zIntLim 

  output := zPro - zDer + zInt 
     
  'if -5000 < zErr AND zErr < 5000 
  '  output := zPro - zDer + zInt
  'else
  '  output :=  zPro - zDer
  '  zInt := 0
  

PUB resetY
  yPro := 0
  yDer := 0
  oldDer_pitch :=0
  yIntInt := 0
  yInt := 0

  
PUB resetX
  xPro := 0
  xDer := 0
  oldDer_roll :=0
  xIntInt := 0
  xInt := 0

PUB resetZ
  zPro := 0
  zDer := 0
  zIntInt := 0
  zInt := 0