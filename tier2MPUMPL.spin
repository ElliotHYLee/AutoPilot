CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

PERCENT_CONST = 1000

OBJ
  sensor    : "Tier1MPUMPL.spin"
  FDS    : "FullDuplexSerial"
  
Var
  '2nd-level analized data
  Long compFilter[3], gForce, heading[3]

  'intermediate data
  Long prevAccX[20], prevAccY[20], prevAccZ[20], avgAcc[3], gyroIntegral[3]

  '1st-level data
  Long acc[3], gyro[3], temperature, mag[3]

PUB main

  FDS.quickStart  
  
  initSensor(15,14)

  setMpu(%000_11_000, %000_01_000) '2000 deg/s, 2g
  
  run

PUB initSensor(scl, sda)
  sensor.initSensor(scl, sda)

PUB setMpu(gyroSet, accSet)
  sensor.setMpu(gyroSet, accSet) 

  
PUB run

    sensor.reportData(@acc, @gyro,@mag, @temperature)
    calcCompFilter_41
{
    FDS.clear
    printSomeX
    fds.newline
    fds.newline
    printSomeY
    fds.newline
    fds.newline
    printAll
    waitcnt(cnt+clkfreq/10)
}
PUB calcCompFilter_41 | a

  a := 970

 getAvgAcc

  gyroIntegral[0] := gyroIntegral[0] - (gyro[1]*50/100)
  compFilter[0] := (a*(compFilter[0] - (gyro[1]*50/100))+500)/PERCENT_CONST + ((PERCENT_CONST-a)*Acc[0]+500)/PERCENT_CONST
 
  
  gyroIntegral[1] := gyroIntegral[1] + (gyro[0]*70/100)  
  compFilter[1] := (a*(compFilter[1] + (gyro[0]*70/100))+500)/PERCENT_CONST + ((PERCENT_CONST-a)*Acc[1]+500)/PERCENT_CONST

  compFilter[2] := acc[2]

PUB calcCompFilter_40 | a         ' gyro set 4 and acc set 0

  a := 970

  getAvgAcc

  gyroIntegral[0] := gyroIntegral[0] - (gyro[1]*130/100)
  compFilter[0] := a*(compFilter[0] - (gyro[1]*130/100))/PERCENT_CONST + ((PERCENT_CONST-a)*avgAcc[0])/PERCENT_CONST

  gyroIntegral[1] := gyroIntegral[1] + (gyro[0]*220/100)  
  compFilter[1] := a*(compFilter[1] + (gyro[0]*220/100))/PERCENT_CONST + ((PERCENT_CONST-a)*avgAcc[1])/PERCENT_CONST

  compFilter[2] := acc[2]

  
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

PUB getHeading(headingPtr)| i
  repeat i from 0 to 2
    Long[headingPtr][i] := heading[i]
PUB getTemperautre(dataPtr)
  Long[dataPtr] := temperature


PUB getEulerAngle(eAnglePtr) | i
  repeat i from 0 to 1
    Long[eAnglePtr][i] := compFilter[i]
    Long[eAnglePtr][2] := avgAcc[2]
  return
PUB getAltitude

PUB getAcc(accPtr) | i
  repeat i from 0 to 1
    Long[accPtr][i] := acc[i]
  return
PUB getGyro(gyroPtr) | i
  repeat i from 0 to 1
    Long[gyroPtr][i] := gyro[i]
  return
PUB magX
  return mag[0]
PUB magY
  return mag[1]
PUB magZ
  return mag[2]  

PRI printSomeX| i, j 

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
        FDS.decLn(mag[j])
  FDS.Str(String("Tempearture = "))
  FDS.decLn(temperature)
  FDS.Str(String("gForce = "))
  FDS.decLn(gForce)