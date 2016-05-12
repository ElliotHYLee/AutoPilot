CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

var

  long prev_dist_ground, curr_dist_ground, target_dist, vel


  'longitudinal nav (x axis)
  long prev_dist_x, curr_dist_x, target_dist_x, velX
  long xkp, xkd, xki
  long xerror, xpro, xdir, xint, xintBound, xOutput

  'translational nav (y axis)
  long prev_dist_y, curr_dist_y, target_dist_y, velY
  long ykp, ykd, yki
  long yerror, ypro, ydir, yint, yintBound, yOutput
  
  
  
  'altitude nav
  long zkp, zkd_up, zkd_down, zki
  long zerror, zpro, zdir, zint, zintBound, pwm

  

PUB calculatePitchAngle(distX, td) | maxOutput

  maxOutput:= 600

  xKp := 1
  xKd := 16
  xKi := 0

  prev_dist_x := curr_dist_x
  curr_dist_x := distX

  xError := td - curr_dist_x ' Err>0 when closer than target dist. fall back -> positive pitch

  if (xError > 300) ' when err is larger than 30 cm
    xError := 300
  elseif (xError < -300)
    xError := -300
  
  xPro := xError*xKp

  xDir := (curr_dist_x - prev_dist_x)*xkd

  xInt := 0
  
  xOutput := xPro - xDir + xInt


  
  'if (xOutput > maxOutput)  'larger than 6 degree
  '  xOutput := maxOutput
  'elseif(xOutput < -maxOutput)
  '  xOutput := -maxOutput
   
  
  return xOutput

PUB calculateRollAngle(distY, td) | maxOutput

  maxOutput:= 600

  yKp := 1
  yKd := 64
  yKi := 0

  prev_dist_y := curr_dist_y
  curr_dist_y := distY

  yError := td - curr_dist_y ' Err>0 when closer than target dist. fall back -> positive pitch

  if (yError > 500) ' when err is larger than 30 cm
    yError := 500
  elseif (yError < -500)
    yError := -500
  
  yPro := yError*yKp

  yDir := (curr_dist_y - prev_dist_y)*xkd

  yInt := 0
  
  yOutput := yPro - yDir + yInt


  
  'if (xOutput > maxOutput)  'larger than 6 degree
  '  xOutput := maxOutput
  'elseif(xOutput < -maxOutput)
  '  xOutput := -maxOutput
   
  
  return yOutput



PUB calculateThrottle(dist, td, dt)| zkd


  zkd := 90
  zkd_down :=  zkd * 4096
  zkd_up := zkd
  zki := 15

  curr_dist_ground := dist                                                                   
  target_dist := td

  
  zerror := target_dist - curr_dist_ground


  'proportional
  if zerror > 0
    zpro := zerror*20
  else
    zpro := zerror/2

  'diravitive
  
  if((curr_dist_ground - prev_dist_ground) <0)
    zdir := zkd_down*(curr_dist_ground - prev_dist_ground)'/(dt/clkfreq) ' milimeter per sec   * kd_down
  else
    zdir := zkd_up*(curr_dist_ground - prev_dist_ground)'/(dt/clkfreq) ' milimeter per sec   * kd_down        

  'integral
  zint := zint + zki*zerror/100

  zintBound := 1500
  if (zint > zintBound)
    zint := zintBound
  if (zint < -1*zintBound)
    zint := -1*zintBound

  
  pwm := zpro - zdir + zint

  if (pwm >1600)
    pwm := 1600
    
  if (pwm < 1475)
    pwm := 1475

  prev_dist_ground := curr_dist_ground


  return pwm


pub reSetZ
  zpro :=0
  zdir :=0
  zint := 0