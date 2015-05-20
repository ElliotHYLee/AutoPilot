CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

VAR

long xKpPtr, xKdPtr, xKiPtr, xErr, xPro, xDer, xInt, xIntInt, xOutput
long yKpPtr, yKdPtr, yKiPtr, yErr, yPro, yDer, yInt, yIntInt, yOutput
long zKp, zKd, zKi, zErr, zPro, zDer, zInt, zIntInt, zOutput

long eAngle[3], gyro[3]
OBJ

  math           : "MyMath.spin"
  
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

  zKp := kpPtr
  zKd := kdPtr
  zKi := kiPtr

PUB calcPIDx(targetVal): output ' controlling motor pulse 0 and 2

  xErr := (targetVal- long[eAngle][0]) 
  xPro := (xErr * long[xKpPtr] + math.getSign(xErr)*5000)/10000
  xDer := (long[gyro][1] * long[xKdPtr] + math.getSign(long[gyro][1])*5000)/10000
  xIntInt := (xIntInt + (xErr*long[xKiPtr])/100_000)
  xInt := -500#> (xIntInt)/1000  <# 500   

  if -5000 < xErr AND xErr < 5000 
    output := xPro + xDer + xInt
  else
    output :=  xPro + xDer
    xInt := 0

    
PUB calcPIDy(targetVal): output ' controlling motor pulse 0 and 2

  yErr := (targetVal- long[eAngle][1]) 
  yPro := (yErr * long[yKpPtr] + math.getSign(yErr)*5000)/10000
  yDer := (long[gyro][0] * long[yKdPtr] + math.getSign(long[gyro][0])*5000)/10000
  yIntInt := (yIntInt + (yErr*long[yKiPtr])/100_000)
  yInt := -500#> (yIntInt)/1000  <# 500   

  'if -5000 < yErr AND yErr < 5000 
    output := yPro - yDer + yInt
  'else
  '  output :=  yPro - yDer
  '  yInt := 0

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