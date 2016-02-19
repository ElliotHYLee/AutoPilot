CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

var

  long prev_dist_ground, curr_dist_ground, target_dist, vel
  long kp, kd_up, kd_down, ki
  long error, pro, dir, int, intBound, pwm

PUB calculateThrottle(dist, td, dt)| kd


  kd := 100
  kd_down :=  kd * 10000
  kd_up := kd
  

  curr_dist_ground := dist                                                                   
  target_dist := td

  
  error := target_dist - curr_dist_ground

' if (error> 200)
'    return 1550
 ' if (error < -200)
'    return 1490
  
  'proportional
  if error > 0
    pro := error*800
  else
    pro := error/2

  'diravitive
  
  if((curr_dist_ground - prev_dist_ground) <0)
    dir := kd_down*(curr_dist_ground - prev_dist_ground)'/(dt/clkfreq) ' milimeter per sec   * kd_down
  else
    dir := kd_up*(curr_dist_ground - prev_dist_ground)'/(dt/clkfreq) ' milimeter per sec   * kd_down        

  'integral
  int := int + error/10

  intBound := 1500
  if (int > intBound)
    int := intBound
  if (int < -1*intBound)
    int := -1*intBound

  
  pwm := pro - dir '+ int

  if (pwm >1700)
    pwm := 1700
    
  if (pwm < 1100)
    pwm := 1100

  prev_dist_ground := curr_dist_ground


  return pwm


pub reSet
  pro :=0
  dir :=0
  int := 0