CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

var

  long prev_dist_ground, curr_dist_ground, target_dist, vel


  'longitudinal nav (x axis)
  long prev_dist_x, curr_dist_x, target_dist_x, velX
  long xKpNavPtr, xKdNavPtr, xKiNavPtr
  long xerror, xpro, xder, xint, xintBound, xOutput

  'translational nav (y axis)
  long prev_dist_y, curr_dist_y, target_dist_y, velY
  long yKpNavPtr, yKdNavPtr, yKiNavPtr
  long yerror, ypro, yder, yint, yintBound, yOutput
  
  
  
  'altitude nav
  long zKpNav_ascPtr, zKpNav_descPtr, zKdNav_ascPtr,zKdNav_descPtr, zKiNavPtr
  long zerror, zpro, zder, zint, zintBound, pwm

PUB setNavPIDConst(xKpAdd, xKdAdd, xKiAdd, yKpAdd, yKdAdd, yKiAdd, zKp_ascAdd, zKp_descAdd, zKd_ascAdd, zKd_descAdd, zKiAdd)

  xKpNavPtr := xKpAdd
  xKdNavPtr := xKdAdd
  xKiNavPtr := xKiAdd
  
  yKpNavPtr := yKpAdd
  yKdNavPtr := yKdAdd
  yKiNavPtr := yKiAdd
  
  zKpNav_ascPtr := zKp_ascAdd
  zKpNav_descPtr := zKp_descAdd
  zKdNav_ascPtr := zKd_ascAdd
  zKdNav_descPtr := zKd_descAdd
  zKiNavPtr := zKiAdd

PUB calculatePitchAngle(distX, td) | maxError, maxOutput, integralBound

  maxError:= 1000      ' 1 meter max error
  maxOutput:= 300      ' 1.5 deg max 
  integralBound := 100 ' 3 deg
 
  'xKp := 110      'actual kp = kp/100
  'xKd := 200
  'xKi := 0'5

  prev_dist_x := curr_dist_x
  curr_dist_x := distX

  xError := td - curr_dist_x ' Err>0 when closer than target dist. fall back -> positive pitch

  if (xError > maxError) ' when err is larger than 30 cm
    xError := maxError
  elseif (xError < -maxError)
    xError := -maxError
  
  xPro := xError*long[xKpNavPtr]/100

  xDer := (curr_dist_x - prev_dist_x)*long[xKdNavPtr]     ' vel >0 getting further -> need to nose down

  xInt := xInt + xError*long[xKiNavPtr]/10

  if (xInt > integralBound)
    xInt := integralBound
  elseif(xInt < -integralBound)
    xInt := -integralBound
  
  xOutput := xPro - xDer + xInt

  
  if (xOutput > maxOutput)
    xOutput := maxOutput
  elseif (xOutput < -maxOutput)
    xOutput := -maxOutput
  
  return xOutput

PUB calculateRollAngle(distY, td) | maxError, maxOutput, integralBound

  maxError := 1000
  maxOutput:= 300       ' 1.5 deg max
  integralBound := 100  ' 3 deg max bound
  
  'yKp := 80
  'yKd := 125
  'yKi := 0'5

  prev_dist_y := curr_dist_y
  curr_dist_y := distY

  yError := td - curr_dist_y ' Err < 0 right to the target. Left down( negtative roll reference) 

  if (yError > maxError) ' when err is larger than 30 cm
    yError := maxError
  elseif (yError < -maxError)
    yError := -maxError
  
  yPro := yError*long[yKpNavPtr]/100

  yDer := (curr_dist_y - prev_dist_y)*long[yKdNavPtr]

  yInt := 0'yInt + yError*long[yKiNavPtr]

  if (yInt > integralBound)
    yInt := integralBound
  elseif(xInt < -integralBound)
    yInt := -integralBound
  
  yOutput := yPro - yDer + yInt

  if (yOutput > maxOutput)
    yOutput := maxOutput
  elseif (yOutput < -maxOutput)
    yOutput := -maxOutput
    
  return yOutput



PUB calculateThrottle(dist, td, dt)| zkd

  {
  'zkd := 90
  'zkd_down :=  zkd * 4096
  'zkd_up := zkd
  'zki := 15

  curr_dist_ground := dist                                                                   
  target_dist := td

  
  zerror := target_dist - curr_dist_ground


  'proportional
  if zerror > 0
    zpro := zerror*20
  else
    zpro := zerror/2

  'derivitive
  
  if((curr_dist_ground - prev_dist_ground) <0)
    zder := zkd_down*(curr_dist_ground - prev_dist_ground)'/(dt/clkfreq) ' milimeter per sec   * kd_down
  else
    zder := zkd_up*(curr_dist_ground - prev_dist_ground)'/(dt/clkfreq) ' milimeter per sec   * kd_down        

  'integral
  zint := zint + zki*zerror/100

  zintBound := 1500
  if (zint > zintBound)
    zint := zintBound
  if (zint < -1*zintBound)
    zint := -1*zintBound

  
  pwm := zpro - zder + zint

  if (pwm >1600)
    pwm := 1600
    
  if (pwm < 1475)
    pwm := 1475

  prev_dist_ground := curr_dist_ground


  return pwm
   }

pub reSetZ
  zpro :=0
  zder :=0
  zint := 0


pub reSetX
  xpro :=0
  xder :=0
  xint := 0


pub reSetY
  ypro :=0
  yder :=0
  yint := 0



PUB getErrX
  return xError
PUB getProX
  return xPro
PUB getDerX
  return xDer
PUB getIntX
  return xInt

PUB getErrY
  return yError
PUB getProY
  return yPro
PUB getDerY
  return yDer
PUB getIntY
  return yInt

PUB getErrZ
  return zError
PUB getProZ
  return zPro
PUB getDerZ
  return zDer
PUB getIntZ
  return zInt  