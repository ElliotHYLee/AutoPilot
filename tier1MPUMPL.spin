{{

  ┌──────────────────────────────────────────┐
  │ MPU-9150 demo using my I2C driver        │
  │ Author: Chris Gadd                       │
  │ Copyright (c) 2014 Chris Gadd            │
  │ See end of file for terms of use.        │
  └──────────────────────────────────────────┘
}}

CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

  SCL = 15
  SDA = 14

  ACC_X_CAL = 512
  ACC_Y_CAL = 436
  ACC_Z_CAL = -700
  
' useful assignments:
  mpuAdd = $68
  ak8Add = $0C
  
VAR
 
  '1st-tier data
  long  acc[3], temperature, gyro[3], mag[3], MPU[10]

  'intermediate data
  long  gyroBias[3]
  byte  asax, asay, asaz

  'raw data
  byte  array[14]

OBJ
  I2C : "I2C Spin driver v1.3"
  FDS : "FullDuplexSerial"

PUB Main 
  fds.quickStart
  
  initSensor(SCL, SDA)                                                    

  setMpu(%000_00_00, %000_00_000)

  
  MPU_9150_demo

PUB reportData(accPtr, gyroPtr, magPtr, temPtr)

  getMpu
  getAk8

  Long[accPtr][0] := acc[0]
  Long[accPtr][1] := acc[1]
  Long[accPtr][2] := acc[2]

  Long[gyroPtr][0] := gyro[0]
  Long[gyroPtr][1] := gyro[1]
  Long[gyroPtr][2] := gyro[2]

  Long[magPtr][0] := mag[0]
  Long[magPtr][1] := mag[1]
  Long[magPtr][2] := mag[2]                           

  Long[temPtr] := temperature
    
                                                    
PUB initSensor(sc, sd)

  I2C.start(sc,sd)      

PUB setMpu(gyroSense, accSense) 

  I2C.write(mpuAdd,$6B,$01)                                                        ' take out of sleep and use gyro-x as clock source
  I2C.write(mpuAdd,$37,$02)                                                        ' enable I2C bypass in order to communicate with the magnetometer at address $0C
  I2C.write(ak8Add,$0A,%1111)                                                      ' access the magnetometer Fuse ROM
  I2C.read_page(ak8Add,$10,@asax,3)                                                ' Read the magnetometer adjustment values

  asax := (asax - 128) / 2                                                      ' These values never change so might as well do a partial calculation here
  asay := (asay - 128) / 2                                                      '  equation from datasheet:    Hadj = H x (((ASA - 128) x 0.5 / 128) + 1) 
  asaz := (asaz - 128) / 2                                                      '  microcontroller friendlier: Hadj = (H x (ASA - 128) / 2) / 128 + H

  I2C.write(mpuAdd,$19,$01)                                                        ' Sample rate divider (divide gyro_rate by 1 + x)
  I2C.write(mpuAdd,$1A,%00_000_110)                                                ' Digital low-pass filtering (0 = 8KHz gyro_rate, !0 = 1KHz gyro_rate)
  I2C.write(mpuAdd,$1B,gyroSense)                                                ' Accelerometer sensitivity ±250°/s
  I2C.write(mpuAdd,$1C,accSense)                                                ' Accelerometer sensitivity ±2g

  calcBias

PUB MPU_9150_demo 

  repeat
    fds.clear
    getMpu
    fds.str(String("accX "))
    fds.dec(acc[0])
    fds.str(String("   gyroX "))
    fds.dec(gyro[0])
    fds.str(String("   gyroY "))
    fds.decLn(gyro[1])

    getAk8

PUB calcBias

  repeat 256
    repeat until I2C.read(mpuAdd,$3A) & $01
    I2C.read_page(mpuAdd,$43,@array,6)
    gyroBias[0] += ~array[0] << 8 | array[1]
    gyroBias[1] += ~array[2] << 8 | array[3]
    gyroBias[2] += ~array[4] << 8 | array[5]

  gyroBias[0] ~>= 8
  gyroBias[1] ~>= 8
  gyroBias[2] ~>= 8

PUB getMpu
  if I2C.read(mpuAdd,$3A) & $01                                                    ' wait for new data (based on sampling rate and digital low-pass filtering registers)
    I2C.read_page(mpuAdd,$3B,@array,14)
    I2C.read(mpuAdd,$3B)
    
    MPU[0]  := ~array[0] << 8 | array[1] - ACC_X_CAL                             ' The accelerometer and gyroscope values are read as big-endians
    MPU[1]  := ~array[2] << 8 | array[3] - ACC_Y_CAL
    MPU[2]  := ~array[4] << 8 | array[5] - ACC_Z_CAL

    temperature   := ~array[6] << 8 | array[7] / 340 + 35                              ' Convert Temp into °c, formula if from the register map document

    MPU[4] := ~array[8] << 8 | array[9] - gyroBias[0] 
    MPU[5] := ~array[10] << 8 | array[11] - gyroBias[1]
    MPU[6] := ~array[12] << 8 | array[13] - gyroBias[2]

    Scale_data

    acc[0] := mpu[0]
    acc[1] := mpu[1]
    acc[2] := mpu[2]

    gyro[0] := mpu[4]
    gyro[1] := mpu[5]
    gyro[2] := mpu[6]
    return true

PUB getAk8

  longfill(@mag[0],0,3)                                                          ' Clear the mag registers

  repeat 1                                                                     ' Average 16 samples together
    I2C.write(ak8Add,$0A,$01)                                                      ' Perform single measurement by setting mode to 1 in control register $0A
    repeat until (I2C.read(ak8Add,$02) & $01)                                      ' Check data ready bit in status1 register
    I2C.read_page(ak8Add,$03,@array,6)
    mag[0] += ~array[1] << 8 | array[0]                                          ' The magnetometer values are read as little-endians
    mag[1] += ~array[3] << 8 | array[2]
    mag[2] += ~array[5] << 8 | array[4]

  mag[0] ~>= 4                                                                   ' Divide by 16 while preserving the sign
  mag[1] ~>= 4
  mag[2] ~>= 4

  mag[0] := (mag[0] * asax) / 128 + mag[0]                                         ' Apply the adjustment values 
  mag[1] := (mag[1] * asay) / 128 + mag[1]                                         ' Hadj = (H x (ASA - 128) / 2) / 128 + H     
  mag[2] := (mag[2] * asaz) / 128 + mag[2]


PUB Scale_data

MPU[0] ^= MPU[1]          'Xor swap x-y axes, negate x axis
MPU[1] := MPU[0]^MPU[1]
MPU[0] := -(MPU[0]^MPU[1]) 

MPU[4] ^= MPU[5]          'Xor swap x-y axes, negate x axis
MPU[5] := MPU[4]^MPU[5]
MPU[4] := -(MPU[4]^MPU[5])


MPU[4] -= gyroBias[0]
MPU[5] -= gyroBias[1]
MPU[6] -= gyroBias[2]    

M_MULT_INT(@MPU[0],@ACCEL_CAL,@MPU[0],@ACCEL_OFF,15)
M_MULT_INT(@MPU[7],@MAG_CAL,@MPU[7],@MAG_OFF,15)



PUB M_MULT_INT(M1,M2,M3,M4,shift)  | RTEMP[3]

RTEMP[0] := ((LONG[M1][0]*LONG[M2][0]+LONG[M1][1]*LONG[M2][3]+LONG[M1][2]*LONG[M2][6])~>shift)-LONG[M4][0]
RTEMP[1] := ((LONG[M1][0]*LONG[M2][1]+LONG[M1][1]*LONG[M2][4]+LONG[M1][2]*LONG[M2][7])~>shift)-LONG[M4][1]
RTEMP[2] := ((LONG[M1][0]*LONG[M2][2]+LONG[M1][1]*LONG[M2][5]+LONG[M1][2]*LONG[M2][8])~>shift)-LONG[M4][2]
longmove(M3,@RTEMP,3)

DAT

GYRO_CAL      long      33250,321,-211,-373,33092,-396,206,109,33440      
ACCEL_CAL     long      33006,-175,172,186,32889,84,-112,0,32629'33007,0,0,11,32890,0,62,83,32629
ACCEL_OFF     long      34,35,63'34,35,63      
MAG_CAL       long      54181,0,0,-7664,51226,0,7208,-10321,50167
MAG_OFF       long      -79,123,-33

                    
{{
┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                                   TERMS OF USE: MIT License                                                  │                                                            
├──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation    │ 
│files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,    │
│modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software│
│is furnished to do so, subject to the following conditions:                                                                   │
│                                                                                                                              │
│The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.│
│                                                                                                                              │
│THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          │
│WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         │
│COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   │
│ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
}}                                        