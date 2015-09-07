CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

PERCENT_CONST = 1000

OBJ
  sensor    : "Tier1MPUMPL_Raw.spin"
  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
Var
  '2nd-level analized data
  Long compFilter[3], gForce, angVel[3], heading[3],avgAcc[3]

  'intermediate data
  Long prevAccX[20], prevAccY[20], prevAccZ[20], avgAccInter[3], gyroIntegral[3]
  Long prevMagX[20], prevMagY[20], prevMagZ[20], avgMag[3]
  
  '1st-level data
  Long acc[3], gyro[3], temperature, mag[3]

  'program variable
  byte compFilterType
  long runStack[128], playID, displayStack[128]

  long before, elapse, base1, base2, base3, flag


  long dt, prev
  
PUB main

  FDS.quickStart  
  
  initSensor(15,14)

  setMpu(%000_00_000, %000_00_000) '250 deg/s, 2g

  startPlay

  repeat
    fds.clear 
    if (flag ==1 AND(cnt > base1 + clkfreq/50))
      printAcc_GCS
      base1 := cnt
      flag :=2
    if (flag==2 AND (cnt > base2 + clkfreq/50))
      printGyro_GCS
      base2 := cnt
      flag := 3
    if (flag ==3 AND (cnt > base3 + clkfreq/50))
      printMag_GCS
      base3 := cnt
      flag := 1


    printAll
'    printMag
    
    
    printDt
    waitcnt(cnt + clkfreq/10)
    
    'fds.newline  
        
      'printCompFilter
   
      'printAll
  '    printSomeX
   
  '    printSomeY
   
      'fds.newline
      'printAll
  {
      fds.newline
      fds.newline
      fds.str(String("compFilter Type: "))
      fds.dec(compFilterType)
      fds.newline
      fds.newline
      fds.decln(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])
   
      'printMagInfo
   
      fds.newline
      fds.newline
      fds.decLn(elapse)
      fds.decLn(clkfreq)
     } 
    



PUB stopPlay
  if playID
    cogstop(playID ~ -1)
    
PUB startPlay
 stopPlay
 playID := cognew(playSensor, @runStack) + 1
 
PUB playSensor
  repeat
    prev := cnt  
    run
    dt := cnt - prev
                     
PUB initSensor(scl, sda)
  sensor.initSensor(scl, sda)

PUB setMpu(gyroSet, accSet)

  sensor.setMpu(gyroSet, accSet) 


PUB run

  sensor.reportData(@acc, @gyro,@mag, @temperature)
           
  getAvgMag


  getAvgAcc

  angVel[0] := gyro[0]' / 131  ' degree per second
  angVel[1] := gyro[1]' / 131  ' degree per second
  angVel[2] := gyro[2]' / 131  ' degree per second    


  heading[0] := avgMag[0] - 20     'magneto meter offset
  heading[1] := avgMag[1] - 25
  heading[2] := avgMag[2] + 5
 
   
PUB getAvgAcc | i, avgCoef

  avgCoef:= 5

  repeat i from 0 to (avgCoef-2)
    prevAccX[i] := prevAccX[i+1]
    prevAccY[i] := prevAccY[i+1]
    prevAccZ[i] := prevAccZ[i+1] 
  prevAccX[avgCoef-1] := acc[0]
  prevAccY[avgCoef-1] := acc[1]
  prevAccZ[avgCoef-1] := acc[2]
    
  avgAccInter[0] := 0
  avgAccInter[1] := 0
  avgAccInter[2] := 0
    
  repeat i from 0 to (avgCoef-1)
    avgAccInter[0] += prevAccX[i]/avgCoef 
    avgAccInter[1] += prevAccY[i]/avgCoef
    avgAccInter[2] += prevAccZ[i]/avgCoef

  avgAcc[0] := avgAccInter[0]
  avgAcc[1] := avgAccInter[1]
  avgAcc[2] := avgAccInter[2]

PUB getAvgMag | i, avgCoef

  avgCoef:= 5

  repeat i from 0 to (avgCoef-2)
    prevMagX[i] := prevMagX[i+1]
    prevMagY[i] := prevMagY[i+1]
    prevMagZ[i] := prevMagZ[i+1] 
  prevMagX[avgCoef-1] := Mag[0]
  prevMagY[avgCoef-1] := Mag[1]
  prevMagZ[avgCoef-1] := Mag[2]
    
  avgMag[0] := 0
  avgMag[1] := 0
  avgMag[2] := 0
    
  repeat i from 0 to (avgCoef-1)
    avgMag[0] += prevMagX[i]/avgCoef 
    avgMag[1] += prevMagY[i]/avgCoef
    avgMag[2] += prevMagZ[i]/avgCoef    


PUB getTemperautre(dataPtr)
  Long[dataPtr] := temperature
        
PUB getAltitude


{
PUB getGyroIntegral(xPtr)| i
  repeat i from 0 to 2
    Long[xPtr][i] := gyroIntegral[i]
  return
}

PUB getAcc(accPtr) | i
  repeat i from 0 to 2
    Long[accPtr][i] := avgAcc[i]
  
  
PUB getGyro(gyroPtr) | i
  Long[gyroPtr][0] := angVel[0]    
  Long[gyroPtr][1] := angVel[1]    
  Long[gyroPtr][2] := angVel[2]    
  
  
PUB getHeading(headingPtr)| i
  Long[headingPtr][0] := heading[0] 
  Long[headingPtr][1] := heading[1] 
  Long[headingPtr][2] := heading[2]

PUB getTemp(tempPtr)
  Long[tempPtr] := temperature
      
PUB magX
  return heading[0]
  
PUB magY
  return heading[1]
  
PUB magZ
  return heading[2]  



PRI printDt

  fds.str(String("dt = "))
  fds.decLn(dt)


  fds.str(String("freq = "))
  fds.dec(80_000_000/dt)
  fds.strLn(String(" Hz"))


PRI printAll_GCS

  printAcc_GCS
  printGyro_GCS
  printMag_GCS

'    repeat i from 0 to 2
'      fds.str(String("[a"))
'      case i
'        0: fds.str(String("x"))
'        1: fds.str(String("y"))
'        2: fds.str(String("z"))
'      fds.dec(avgAcc[i])
'      fds.str(String("]"))
'     
'      fds.str(String("[g"))
'      case i
'        0: fds.str(String("x"))
'        1: fds.str(String("y"))
 '       2: fds.str(String("z")) 
  '    fds.dec(angVel[i])
  '    fds.str(String("]"))
   '  
   '   fds.str(String("[q"))
   ''   case i
   '     0: fds.str(String("x"))
   '     1: fds.str(String("y"))
   '     2: fds.str(String("z")) 
   '   fds.dec(heading[i])
   '   fds.str(String("]"))

PRI printAcc_GCS | i

  repeat i from 0 to 2
    fds.str(String("[g"))
    case i
      0: fds.str(String("x"))
      1: fds.str(String("y"))
      2: fds.str(String("z")) 
    fds.dec(angVel[i])
    fds.str(String("]"))

PRI printMag_GCS | i

  repeat i from 0 to 2
    fds.str(String("[q"))
    case i
      0: fds.str(String("x"))
      1: fds.str(String("y"))
      2: fds.str(String("z")) 
    fds.dec(heading[i])
    fds.str(String("]"))
    
PRI printGyro_GCS | i

  repeat i from 0 to 2
    fds.str(String("[a"))
    case i
      0: fds.str(String("x"))
      1: fds.str(String("y"))
      2: fds.str(String("z"))
    fds.dec(avgAcc[i])
    fds.str(String("]"))
  
PRI printMagInfo| i, j

  fds.strLn(String("Euler Angle"))
  fds.str(String("X: "))
  fds.dec(compFilter[0])
  fds.str(String(" Y: "))
  fds.dec(compFilter[1])
  fds.str(String(" Z: "))
  fds.decLn(compFilter[2])
  fds.newline
  fds.strLn(String("Avg Magnetometer"))  
  fds.str(String("X: "))
  fds.dec(avgMag[0])
  fds.str(String(" Y: "))
  fds.dec(avgMag[1])
  fds.str(String(" Z: "))
  fds.decLn(avgMag[2])
  fds.newline
  fds.str(string("magnitude^2 of magnetometer: "))
  fds.decLn(avgMag[0]*avgMag[0] + avgMag[1]*avgMag[1] + avgMag[2]*avgMag[2])

  fds.newline  
  fds.str(String("x/y (aTan)"))
  fds.decLn(avgMag[0]/avgMag[1])


PRI printMag| i, j


  fds.strLn(String("Avg Magnetometer"))  
  fds.str(String("X: "))
  fds.dec(mag[0])
  fds.str(String(" Y: "))
  fds.dec(mag[1])
  fds.str(String(" Z: "))
  fds.decLn(mag[2])
  fds.newline
  fds.str(string("magnitude^2 of magnetometer: "))
  fds.decLn(avgMag[0]*avgMag[0] + avgMag[1]*avgMag[1] + avgMag[2]*avgMag[2])

 ' fds.newline  
 ' fds.str(String("x/y (aTan)"))
 ' fds.decLn(avgMag[0]/avgMag[1])
  
PRI printSomeX| i, j 
  fds.dec(gyro[1])
  fds.strLn(String("   gyro"))
  fds.dec(acc[0])
  fds.strLn(String("   AccX"))
'  fds.dec(avgAcc[0])
'  fds.strLn(String("   avgAccX"))
  fds.dec(gyroIntegral[0])
  fds.strLn(String("   gyroIntegral"))       
  fds.dec(compFilter[0])
  fds.str(String("   compFilter X"))
  fds.newline
  fds.dec((avgAcc[0] - compFilter[0])*90/9800)
  fds.strLn(String("   Deg_err_compX"))
  fds.dec((avgAcc[0] - gyroIntegral[0])*90/9800)
  fds.strLn(String("   Deg_err_gyroIntegralX"))

PRI printSomeY| i, j 
                                                                
  fds.dec(acc[1])
  fds.strLn(String("   AccY"))
'  fds.dec(avgAcc[1])
'  fds.strLn(String("   avgAccY"))
  fds.dec(gyroIntegral[1])
  fds.strLn(String("   gyroIntegral"))
  fds.dec(compFilter[1])
  fds.str(String("   compFilter Y"))
  fds.newline
  fds.dec((avgAcc[1] - compFilter[1])*90/9800 )
  fds.strLn(String("   Deg_err_compX"))
  fds.dec((avgAcc[1] - gyroIntegral[1])*90/9800 )
  fds.strLn(String("   Deg_err_gyroIntegralX"))

PRI printCompFilter
  fds.str(String("cx: "))
  fds.decLn(compFilter[0])
  fds.str(String("cy: "))
  fds.decLn(compFilter[1])
  fds.str(String("cz: "))
  fds.decLn(compFilter[2])

  'if (compFilter[2] < 80000 )
 '   waitcnt(cnt + clkfreq*5)

          
PRI printAll | i, j
  repeat i from 0 to 2
    repeat j from 0 to 2
      if i==0
        FDS.str(String("Acc["))
        FDS.dec(j)
        FDS.str(String("]=  "))      
        FDS.dec(acc[j])
        FDS.str(String(" AvgAcc["))
        FDS.dec(j)
        FDS.str(String("]=  "))      
        FDS.decLn(avgAcc[j])
        
        FDS.str(String("Comp["))
        FDS.dec(j)
        FDS.str(String("]= "))      
        FDS.dec(compFilter[j])
        FDS.str(String("  err_degree "))
        FDS.decLn( -(avgAcc[j]-compFilter[j])*90/9800    )
      if i==1
        FDS.str(String("Gyro["))
        FDS.dec(j)
        FDS.str(String("]= "))      
        FDS.decLn(gyro[j])
      if i ==2
        FDS.str(String("Mag["))
        FDS.dec(j)
        FDS.str(String("]= "))      
        FDS.decLn(heading[j])
    'fds.decLn(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2])
  FDS.Str(String("Tempearture = "))
  FDS.decLn(temperature)
  FDS.Str(String("gForce = "))
  FDS.decLn(gForce)