CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

VAR

long xKpPtr, xKdPtr, xKiPtr, xErr, xPro, xDer, xInt, xIntInt
long yKp, yKd, yKi, yErr, yPro, yDer, yInt, yIntInt, yOutput
long zKp, zKd, zKi, zErr, zPro, zDer, zInt, zIntInt, zOutput

long eAngle[3], gyro[3]

OBJ

  math           : "MyMath.spin"  

PUB getErr

  return xErr

PUB getPro

  return xPro
PUB getDer
  return xDer
PUB getInt
  return xInt
               
PUB setAttVal(eAnglePtr, gyroPtr)

 eAngle := eAnglePtr
 gyro := gyroPtr

PUB setXaxis(kpPtr, kdPtr, kiPtr)

  xKpPtr := kpPtr
  xKdPtr := kdPtr
  xKiPtr := kiPtr

PUB setYaxis(kpPtr, kdPtr, kiPtr, outputPtr)

  yKp := kpPtr
  yKd := kdPtr
  yKi := kiPtr
  yOutput := outputPtr

PUB setZaxis(kpPtr, kdPtr, kiPtr, outputPtr)

  zKp := kpPtr
  zKd := kdPtr
  zKi := kiPtr
  zOutput := outputPtr

PUB calcPIDx(targetVal): output ' controlling motor pulse 0 and 2

  xErr := (targetVal- long[eAngle][0]) 
  xPro := (xErr * long[xKpPtr] + math.getSign(xErr)*5000)/10000
  xDer := (long[gyro][1] * long[xKdPtr] + math.getSign(long[gyro][1])*5000)/10000
  xIntInt := (xIntInt + (xErr*long[xKiPtr])/1_000_000)
  xInt := -20#> (xIntInt)/1000  <# 20   

  if -5000 < xErr AND xErr < 5000 
    output := xPro + xDer + xInt
  else
    output :=  xPro + xDer
    xInt := 0

'  long[pulsePtr][0] := 1200 #> 1250 + output  <# 1600      
'  long[pulsePtr][2] := 1200 #> 1250 - output  <# 1600

  
{{ 
PUB pidXAxis(axis)| pMotor, nMotor, dEdt
  
  nMotor := axis       ' motot 0  - negative tilt 
  pMotor := axis + 2   ' motor 2  - positive tilt

  if (axis==0)
    dEdt := gyro[1]
  else
    dEdt := gyro[0]
    
  error := (targetEAngle[axis] - eAngle[axis])

  proportional := (error * kp + math.getSign(error)*5000)/10000

  derivative := (dEdt * kd + math.getSign(dEdt)*5000)/10000  

  integral_intermediate := (integral_intermediate + (error*ki)/1_000_000)
  integral[0] := -20#> (integral_intermediate)/1000  <# 20

  curTime := cnt  
  tElapse := (curTime - prevTime )*1000000/clkfreq  'micro second
  
  prevTime := curTime

  
  if -5000 < error AND error < 5000 
    outPut := proportional + derivative + integral[0]
  else
    outPut := proportional + derivative
    integral[0] := 0
   
  pulse[pMotor] := 1200 #> 1250 - outPut  <# 1600
  pulse[nMotor] := 1200 #> 1250 + outPut  <# 1600

  }}       