CON
        
  _clkmode = xtal1 + pll16x     
  _xinfreq = 6_250_000


VAR

  long prevLcX, xErr, xPro, xDer, xIntInt, xInt, xOutput
  long prevLcY, yErr, yPro, yDer, yIntInt, yInt, yOutput
  long prevLcZ, zErr, zPro, zDer, zIntInt, zInt, zOutput
  

PUB xCalc(Kp, Kd, Ki, lcX, tLcX, proPtr, derPtr, intPtr, outputPtr) | maxInt, vel

  maxInt := 50            '3 degree
  
  xErr := (tLcX - lcX)
  xPro := xErr * Kp /100

  vel := lcX- prevLcX
  xDer := vel*kd
  prevLcX := lcX
  
  xIntInt := (xIntInt + xErr*Ki/100)
  
  if (xIntInt < -maxInt)
    xIntInt := -maxInt
  elseif (xIntInt > maxInt)
    xIntInt := maxInt
  xInt := xIntInt 

  
  if -500 < xErr AND xErr < 500       ' 50 cm meter limit for error
    xOutput := xPro - xDer + xInt
  else
    xOutput := xPro - xDer
    xIntInt := 0

  xOutput := -300 #> xOutput <# 300              

  long[proPtr] := xPro
  long[derPtr] := xDer
  long[intPtr] := xInt
  long[outputPtr] := xOutput 

PUB xReset

  xPro:=0
  xDer :=  0
  xInt := 0
  xOutput :=0    

PUB yCalc(Kp, Kd, Ki, lcY, tLcY, proPtr, derPtr, intPtr, outputPtr) | maxInt, vel   

  maxInt := 50

  yErr := tLcY - lcY
  yPro := yErr * Kp   /100

  vel := lcY - prevLcY
  yDer := vel*Kd
  prevLcY := lcY


  if (Ki== 0)
    yIntInt := 0
  else
    yIntInt :=  (yIntInt + yErr*Ki/100)     


  if (yIntInt < -maxInt)
    yIntInt := -maxInt
  elseif (yIntInt > maxInt)
    yIntInt := maxInt
   
  yInt := yIntInt 

  yOutput := yPro - yDer + yInt
  
  yOutput := -300 #> yOutput <# 300
  
  long[proPtr] := yPro
  long[derPtr] := yDer
  long[intPtr] := yInt
  long[outputPtr] := yOutput

PUB yReset

  yPro:=0
  yDer :=  0
  yInt := 0
  yOutput :=0

PUB zCalc(Kp_asc, Kp_des, Kd_asc, Kd_des, Ki, lcZ, tLcZ, proPtr, derPtr, intPtr, outputPtr) | maxInt, vel

  zErr := tLcZ - lcZ     ' pos error = neet to go up (kinect at gnd)

  if zErr > 0
    zPro := zErr*Kp_asc /10   ' 512 previously
  else
    zPro := zErr*Kp_des /10    '2 previously

  vel := lcZ - prevLcZ
    
  if vel < 0 
    zDer := kd_des*vel      '/(dt/clkfreq) ' milimeter per sec   * kd_down
  else
    zDer := kd_asc*vel        '/(dt/clkfreq) ' milimeter per sec   * kd_down

  prevLcZ := lcZ

  zOutput :=  -100 #> zPro - zDer <# 100 

  
  long[proPtr] := zPro
  long[derPtr] := zDer
  long[intPtr] := zInt
  long[outputPtr] := zOutput
    
PUB zReset

  zPro:=0
  zDer :=  0
  zInt := 0
  zOutput :=0

  