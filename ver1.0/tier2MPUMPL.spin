CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

PERCENT_CONST = 1000

OBJ
  sensor    : "Tier1MPUMPL.spin"
  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
Var
  '2nd-level analized data
  Long compFilter[3], gForce, heading[3]

  'intermediate data
  Long prevAccX[20], prevAccY[20], prevAccZ[20], avgAcc[3], gyroIntegral[3]
  Long prevMagX[20], prevMagY[20], prevMagZ[20], avgMag[3]
  
  '1st-level data
  Long acc[3], gyro[3], temperature, mag[3]

  'program variable
  byte compFilterType
  long runStack[128], playID, displayStack[128]

  long before, after, elapse
  
  
PUB main

  FDS.quickStart  
  
  initSensor(15,14)

  setMpu(%000_11_000, %000_01_000) '2000 deg/s, 4g

  startPlay

  repeat

    FDS.clear
    printCompFilter

'    printSomeX

'    printSomeY

    fds.newline
    'printAll
{
    fds.newline
    fds.newline
    fds.str(String("compFilter Type: "))
    fds.dec(compFilterType)
    fds.newline
    fds.newline
    fds.decln(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])
 }
    printMagInfo

    fds.newline
    fds.newline
    fds.decLn(elapse)
    fds.decLn(clkfreq) 
    waitcnt(cnt+clkfreq/10)




PUB stopPlay
  if playID
    cogstop(playID ~ -1)
    
PUB startPlay
 stopPlay
 playID := cognew(playSensor, @runStack) + 1
 
PUB playSensor
  repeat
    'before := cnt
    run
    'elapse := cnt - before
    
                     
PUB initSensor(scl, sda)
  sensor.initSensor(scl, sda)

PUB setMpu(gyroSet, accSet)

  sensor.setMpu(gyroSet, accSet) 
  if (gyroSet==%000_11_000) AND (accSet==%000_00_000)
    compFilterType := 12
  elseif ((gyroSet==%000_11_000) AND (accSet==%000_01_000))
    compFilterType := 13     
  elseif (gyroSet==%000_11_000) AND (accSet==%000_10_000)
    compFilterType := 14     
  elseif (gyroSet==%000_11_000) AND (accSet==%000_11_000)
    compFilterType := 15     
  else
    compFilterType := 12


PUB run

  sensor.reportData(@acc, @gyro,@mag, @temperature)

  if compFilterType == 12
    calcCompFilter_30
  elseif compFilterType == 13
    calcCompFilter_31
  else
    calcCompFilter_30  'default

  getAvgMag
  
      
PUB calcCompFilter_31 | a,tempX,tempY,tempZ, tempTot
{{
calcCompFilter_31
Complementary Filter for 2000 deg/s and 4g
}}
  a := 970

  getAvgAcc

  gyroIntegral[0] := gyroIntegral[0] - (gyro[1]*27/100)
  compFilter[0] := (a*(compFilter[0] - (gyro[1]*27/100))+500)/PERCENT_CONST + ((PERCENT_CONST-a)*avgAcc[0]+500)/PERCENT_CONST
 
  
  gyroIntegral[1] := gyroIntegral[1] + (gyro[0]*26/100)  
  compFilter[1] := (a*(compFilter[1] + (gyro[0]*26/100))+500)/PERCENT_CONST + ((PERCENT_CONST-a)*avgAcc[1]+500)/PERCENT_CONST

  tempX := (compFilter[0] * compFilter[0])/100
  tempY := (compFilter[1] * compFilter[1])/100
  tempTot := 680000 - tempX - tempY
  compFilter[2] := 10*math.sqrt(tempTot)

'  compFilter[2] := acc[2]

PUB calcCompFilter_30 | a,tempX,tempY,tempZ, tempTot           ' gyro set 4 and acc set 0
{{
calcCompFilter_30
Complementary Filter for 2000 deg/s and 2g
}}
  a := 970

  getAvgAcc

  gyroIntegral[0] := gyroIntegral[0] - (gyro[1]*37/100)
  compFilter[0] := a*(compFilter[0] - (gyro[1]*37/100))/PERCENT_CONST + ((PERCENT_CONST-a)*avgAcc[0])/PERCENT_CONST

  gyroIntegral[1] := gyroIntegral[1] + (gyro[0]*50/100)  
  compFilter[1] := a*(compFilter[1] + (gyro[0]*50/100))/PERCENT_CONST + ((PERCENT_CONST-a)*avgAcc[1])/PERCENT_CONST
  
  tempX := (compFilter[0] * compFilter[0]+5000)/10000
  tempY := (compFilter[1] * compFilter[1]+5000)/10000
  tempTot := 6800 - tempX - tempY
  compFilter[2] := 100*math.sqrt(tempTot)

  
PUB getAvgAcc | i, avgCoef

  avgCoef:= 5

  repeat i from 0 to (avgCoef-2)
    prevAccX[i] := prevAccX[i+1]
    prevAccY[i] := prevAccY[i+1]
    prevAccZ[i] := prevAccZ[i+1] 
  prevAccX[avgCoef-1] := acc[0]
  prevAccY[avgCoef-1] := acc[1]
  prevAccZ[avgCoef-1] := acc[2]
    
  avgAcc[0] := 0
  avgAcc[1] := 0
  avgAcc[2] := 0
    
  repeat i from 0 to (avgCoef-1)
    avgAcc[0] += prevAccX[i]/avgCoef 
    avgAcc[1] += prevAccY[i]/avgCoef
    avgAcc[2] += prevAccZ[i]/avgCoef

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

PUB getHeading(headingPtr)| i
  repeat i from 0 to 2
    Long[headingPtr][i] := avgMag[i]
    
PUB getTemperautre(dataPtr)
  Long[dataPtr] := temperature
        
PUB getEulerAngle(eAnglePtr)

  Long[eAnglePtr][0] := compFilter[0]
  Long[eAnglePtr][1] := compFilter[1]
  Long[eAnglePtr][2] := compFilter[2]
  return
  
PUB getAltitude


PUB getGyroIntegral(xPtr)| i
  repeat i from 0 to 2
    Long[xPtr][i] := gyroIntegral[i]
  return



PUB getAcc(accPtr) | i
  repeat i from 0 to 2
    Long[accPtr][i] := acc[i]
  return
  
PUB getGyro(gyroPtr) | i
  Long[gyroPtr][0] := gyro[0]
  Long[gyroPtr][1] := gyro[1]
  Long[gyroPtr][2] := gyro[2]
  return
  
PUB magX
  return mag[0]
PUB magY
  return mag[1]
  
PUB magZ
  return mag[2]  


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
  fds.str(string("magnitude of magnetometer: "))
  fds.decLn(avgMag[0]*avgMag[0] + avgMag[1]*avgMag[1] + avgMag[2]*avgMag[2])

  fds.newline  
  fds.str(String("x/y (aTan)"))
  fds.decLn(avgMag[0]/avgMag[1])
  
  
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
        FDS.decLn(avgMag[j])
    'fds.decLn(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2])
  FDS.Str(String("Tempearture = "))
  FDS.decLn(temperature)
  FDS.Str(String("gForce = "))
  FDS.decLn(gForce)