CON
        
  _clkmode = xtal1 + pll16x     
  _xinfreq = 6_250_000


var

  long pitchErr, pitchPro, pitchDer, pitchIntInt, pitchInt, pitchOutput
  long rollErr, rollPro, rollDer, rollIntInt, rollInt, rollOutput
  long yawErr, yawPro, yawDer, yawIntInt, yawInt, yawOutput, yawFreqTimer

PUB pitch(Kp, Kd, Ki, theta, targetTheta, angVel, proPtr, derPtr, intPtr, outputPtr) | maxInt

  maxInt := 50_000
  
  pitchErr := (targetTheta - theta)
  pitchPro := pitchErr * Kp /10_000

  angVel /= 131
  pitchDer := angVel*kd/10_000

  pitchIntInt := (pitchIntInt + pitchErr*Ki/100_000)
  
  if (pitchIntInt < -maxInt)
    pitchIntInt := -maxInt
  elseif (pitchIntInt > maxInt)
    pitchIntInt := maxInt
  pitchInt := pitchIntInt/1000 

  
  if -5000 < pitchErr AND pitchErr < 5000
    pitchOutput := pitchPro - pitchDer + pitchInt
  else
    pitchOutput := pitchPro - pitchDer
    pitchIntInt := 0                

  long[proPtr] := pitchPro
  long[derPtr] := pitchDer
  long[intPtr] := pitchInt
  long[outputPtr] := pitchOutput  

PUB resetPitch

  pitchErr  := 0
  pitchPro    := 0 
  pitchDer    := 0 
  pitchIntInt    := 0 
  pitchOutput := 0 

PUB roll(Kp, Kd, Ki, phi, targetPhi, angVel, proPtr, derPtr, intPtr, outputPtr) | maxInt

  maxInt := 10_000

  rollErr := (targetPhi - phi)
  rollPro := rollErr * Kp /10_000

  angVel /= 131
  rollDer := angVel*kd/10_000

  rollIntInt := (rollIntInt + rollErr*Ki/100_000)
  
  if (rollIntInt < -maxInt)
    rollIntInt := -maxInt
  elseif (rollIntInt > maxInt)
    rollIntInt := maxInt
  rollInt := rollIntInt/1000 

  
  if -5000 < rollErr AND rollErr < 5000
    rollOutput := rollPro - rollDer + rollInt
  else
    rollOutput := rollPro - rollDer
    rollInt := 0

  long[proPtr] := rollPro
  long[derPtr] := rollDer
  long[intPtr] := rollInt
  long[outputPtr] := rollOutput


pub resetRoll

  rollErr := 0
  rollPro := 0
  rollDer := 0
  rollIntInt :=0
  rollInt :=0
  rollOutput :=0

PUB yaw(Kp, Kd, Ki, psi, targetPsi, angVel, proPtr, derPtr, intPtr, outputPtr) | outputLim,angLim

 outputLim := 150
 angLim := 1000
 
 if (true)'(cnt-yawFreqTimer => clkfreq/101)
   yawErr := (targetPsi - psi)
   
  { if (yawErr =>18000)
     yawErr := yawErr - 36000
   elseif (yawErr < -18000)     
     yawErr := yawErr + 3600      }
   
  { if (yawErr > angLim)
     yawErr := angLim
   elseif(yawErr <-angLim)
     yawErr := -angLim       }
   
   yawPro := yawErr*Kp/10_000
   
   yawDer := angVel*Kd/10_000
   
   yawIntInt := yawIntInt + yawErr*Ki/100_000
   
   if (yawIntInt> 50000)
     yawIntInt := 50000
   elseif (yawIntInt <-50000)
     yawIntInt := -50000
   
   yawInt := yawIntInt/1000
    
   yawOutput := yawPro - yawDer + yawInt

   if (yawOutput > outputLim)
     yawOutput := outputLim
   elseif(yawOutput < -outputLim)
     yawOutput := -outputLim     
   yawFreqTimer := cnt
 else
   yawOutput:=0

 long[proPtr] := yawPro
 long[derPtr] := yawDer
 long[intPtr] := yawInt
 long[outputPtr] := yawOutput   

PUB resetYaw

  yawErr := 0
  yawPro := 0
  yawDer := 0
  yawInt := 0
  yawOutput := 0

  