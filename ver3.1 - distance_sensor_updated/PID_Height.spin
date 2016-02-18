CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

var

  long prev_dist_ground, curr_dist_ground, target_dist, vel
  long kp, kd_up, kd_down, ki
  long error, pro, dir, int, intBound, pwm

PUB calculateThrottle(dist, td, dt)

  kd_down :=  16
  kd_up := 20
  

  curr_dist_ground := dist
  target_dist := td

  
  error := target_dist - curr_dist_ground

  'proportional
  pro := error*2

  'diravitive
  vel := (curr_dist_ground - prev_dist_ground)*dt/clkfreq ' milimeter per sec
  if(vel <=0)
    dir := vel * kd_down
  else
    dir := vel * kd_up

  'integral
  int := int + 5*error

  intBound := 1600
  if (int > intBound)
    int := intBound
  if (int < -1*intBound)
    int := -1*intBound

  
  pwm := pro - dir + int

  if (pwm >1600)
    pwm := 1600
    
  if (pwm < 1200)
    pwm := 1200

  prev_dist_ground := curr_dist_ground


  return pwm
    