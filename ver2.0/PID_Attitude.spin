CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

VAR

long xKpPtr, xKdPtr, xKiPtr, xErr, xPro, xDer, xInt, xIntInt, xOutput
long yKpPtr, yKdPtr, yKiPtr, yErr, yPro, yDer, yInt, yIntInt, yOutput
long zKpPtr, zKdPtr, zKiPtr, zErr, zPro, zDer, zInt, zIntInt, zOutput

long eAngle[3], gyro[3]

long oldDer_roll

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
  return yDer
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

PUB calcPIDRoll(targetVal): output| alpha ' controlling motor pulse 0 and 2

  
  xErr := (targetVal- long[eAngle][1])  'eAngle[1] = roll
  xPro :=  (xErr * long[xKpPtr])/10000
  alpha := long[xKdPtr]
  oldDer_roll := (long[gyro][1]*alpha )/1000 + (oldDer_roll*(1000 - alpha))/1000
  xDer := (oldDer_roll)/10   '(long[gyro][1] * long[xKdPtr] )/10000
  xIntInt := (xIntInt + (xErr*long[xKiPtr])/100_000)
  xInt := -500 #> (xIntInt)/1000  <# 500   


   
    if -5000 < xErr AND xErr < 5000 
      output := (xPro - xDer + xInt)
    else
      output :=  xPro - xDer
      xInt := 0

PUB calcPIDRoll2(targetVal): output ' controlling motor pulse 0 and 2

{
  xErr := (targetVal- long[eAngle][1])  'eAngle[1] = roll
  xPro := (xErr * long[xKpPtr])/10000
  xDer := (long[gyro][1] * long[xKdPtr] )/10000
 'xIntInt := (xIntInt + (xErr*long[xKiPtr])/100_000)
  'xInt := -100 #> (xIntInt)/1000  <# 100   


  if xErr > 100     
    if -5000 < xErr AND xErr < 5000 
      output := (xPro + xDer + xInt)
    else
      output :=  xPro + xDer
      xInt := 0
}
    
PUB calcPIDPitch(targetVal): output  ' controlling motor pulse 0 and 2  - pitch control

  yErr := (targetVal- long[eAngle][0]) 'eAngle[0] = pitch
  'yPro := (yErr * long[yKpPtr] + math.getSign(yErr)*5000)/10000
  yPro := (yErr * long[yKpPtr] )/10000    
  'yDer := (long[gyro][0] * long[yKdPtr] + math.getSign(long[gyro][0])*5000)/10000
  yDer := (long[gyro][0] * long[yKdPtr] )/10000
  yIntInt := (yIntInt + (yErr*long[yKiPtr])/100_000)


  yInt := -200 #> (yIntInt)/1000  <# 200   

  if yErr > 100  
    if -5000 < yErr AND yErr < 5000 
      output := yPro - yDer + yInt
    else
      output :=  yPro - yDer
      yInt := 0

PUB calcPIDYaw(targetVal): output

  zErr := (targetVal- long[eAngle][2]) 
  zPro := (zErr * long[zKpPtr] + math.getSign(zErr)*5000)/10000
  zDer := (long[gyro][2] * long[zKdPtr] + math.getSign(long[gyro][2])*5000)/10000
  zIntInt := (zIntInt + (zErr*long[zKiPtr])/100_000)

  zInt := -50 #> (zIntInt)/1000  <# 50   

  if -5000 < zErr AND zErr < 5000 
    output := zPro - zDer + zInt
  else
    output :=  zPro - zDer
    zInt := 0
  

PUB resetY
  yPro := 0
  yDer := 0
  yIntInt := 0
  yInt := 0

  
PUB resetX
  xPro := 0
  xDer := 0
  xIntInt := 0
  xInt := 0

PUB resetZ
  zPro := 0
  zDer := 0
  zIntInt := 0
  zInt := 0