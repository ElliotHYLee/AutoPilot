CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

VAR

long xKp, xKd, xKi, xErr, xPro, xDer, xInt, xIntInt, xOutput
long yKp, yKd, yKi, yErr, yPro, yDer, yInt, yIntInt, yOutput
long zKp, zKd, zKi, zErr, zPro, zDer, zInt, zIntInt, zOutput

OBJ

  math           : "MyMath.spin"  

PUB setXaxis(kpPtr, kdPtr, kiPtr, outputPtr)

  xKp := kpPtr
  xKd := kdPtr
  xKi := kiPtr
  xOutput := outputPtr

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

PUB calcPIDx(motorPtr1, motorPtr2)






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