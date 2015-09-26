CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

  CMNSCALE = 10_000

OBJ
  sensor : "Tier2MPUMPL_Refinery.spin"
  fds    : "FullDuplexSerial.spin"
  tr     : "TRIG"
  math   : "MyMath"
VAR


  'MPU varaibles
  long t3_acc[3], t3_gyro[3], t3_mag[3], t3_gyroIsUpdated, t3_dcmIsUpdated, t3_accMagIsUpdated
  long t3_mpuCogId, t3_mpuStack[128]
  long t3_dt_mpu, t3_prev_mpu, t3_freq_mpu

  'DCM variables
  long t3_dcmCogId, t3_dcmStack[128]
  long t3_dcm[9], t3_eye[9], t3_imdt[9], t3_imdt2[9], t3_imdt3[9]
  long t3_omega[3], t3_euler[3], t3_acc_body[3], t3_acc_earth[3], t3_mag_earth[3]
  long t3_err_acc_earth[3], t3_err_mag_earth[3], t3_err_body[3], t3_err_earth[3], t3_I[3],t3_intrmdtI[3]  
  long t3_dt_dcm, t3_prev_dcm, t3_freq_dcm, t3_prev_compensation
  
  'first values for dcm
  long t3_avgAcc[3], t3_prevAccX[20], t3_prevAccY[20], t3_prevAccZ[20],  t3_avgAccInter[3] 
  long t3_first_mag[3], t3_first_euler_in[3],t3_first_euler_out[3]
  long t3_first_dcm[9]

  
  'general
  long t3_counter
  
  'DCM anomaly checker
  byte t3_an_skew_omega

  'Debug Window
  long t3_dt_fds, t3_prev_fds
  long t3_acc_d[3], t3_gyro_d[3], t3_mag_d[3]
  long t3_dcm_d[9], t3_eye_d[9], t3_imdt_d[9],t3_imdt2_d[9], t3_imdt3_d[9], t3_first_dcm_d[9]
  long t3_omega_d[3], t3_euler_d[3]
  long t3_matrix_monitor1[9], t3_matrix_monitor2[9], t3_matrix_monitor3[9]   
  long t3_matrix_monitor1_d[9], t3_matrix_monitor2_d[9], t3_matrix_monitor3_d[9]   
  long t3_avg_acc_d[3], t3_first_mag_d[3], t3_first_euler_in_d[3],t3_first_euler_out_d[3] 
  
PUB main

  fds.quickStart  
  
  masterKey_tier3
 ' startDcm

  'turnOnMPU


  
  repeat

    t3_prev_fds := cnt
    freezResult   
    fds.clear
    printDt
    fds.newline
    fds.strln(String("----------------------------"))
    ' first values
    printFirstMag
    fds.newline
    printAvgAcc
    fds.newline 
    printFirstEulerInput
    fds.newline 
    printFisrtDCM
    fds.newline
    printFirstEulerOutput
    fds.strln(String("----------------------------")) 
    ' progressive values
    printCurrentMag
    fds.newline  
    printAcc
    printOmega
    printDCM
    fds.newline
    printEuler
    
    fds.strln(String("----------------------------"))
    ' debugging dcm    
    printMatrixMonitor
    fds.newline
    printAnomalyMonitor
    waitcnt(cnt + clkfreq/60)
    t3_dt_fds := cnt - t3_prev_fds

PUB getDcmStatus 

  result :=  t3_dcmIsUpdated
  t3_dcmIsUpdated:=0

PUB getEulerAngles(xPtr)

  long[xPtr][0] := t3_euler[0]
  long[xPtr][1] := t3_euler[1]
  long[xPtr][2] := t3_euler[2]  

PUB getAcc(xPtr)

  long[xPtr][0] := t3_acc[0]
  long[xPtr][1] := t3_acc[1]
  long[xPtr][2] := t3_acc[2]

PUB getGyro(xPtr)

  long[xPtr][0] := t3_gyro[0]
  long[xPtr][1] := t3_gyro[1]
  long[xPtr][2] := t3_gyro[2]      

PUB getMag(xPtr)

  long[xPtr][0] := t3_mag[0]
  long[xPtr][1] := t3_mag[1]
  long[xPtr][2] := t3_mag[2]      

PUB masterKey_tier3

  turnOnMPU

  setUpDCM
  
  startMpu

  startDCM  


PUB turnOnMPU

  sensor.initSensor(15,14)
  sensor.setMpu(%000_00_000, %000_00_000) '250 deg/s, 2g

PUB stopMpu
  if t3_mpuCogId
    cogstop(t3_mpuCogId ~ -1)
    
PUB startMpu
  stopMpu
  t3_mpuCogId := cognew(runMpu, @t3_mpuStack) + 1

  
PUB runMpu | goCompensation

  goCompensation := 0

  repeat
    t3_prev_mpu := cnt
    sensor.runGyro
    sensor.getGyro(@t3_gyro)

   goCompensation ++

    if goCompensation => 9
      sensor.runAccMag
      sensor.getAcc(@t3_acc)
      sensor.getHeading(@t3_mag)
      t3_accMagIsUpdated := 1   ' for DCM cog
      goCompensation := 0
      
    t3_gyroIsUpdated := 1  

    t3_dt_mpu := cnt - t3_prev_mpu
    

  
PUB setUpDCM | counter


  repeat 100
    sensor.runAccMag  
  sensor.getAcc(@t3_acc)  
  sensor.getHeading(@t3_mag)
  
  repeat counter from 0 to 2
    t3_avgAcc[counter] := t3_acc[counter]
  
  repeat counter from 0 to 2
    t3_I[counter] := 0  
    t3_euler[counter] := 0

    t3_first_mag[counter] := t3_mag[counter]

  'math.acc2ang(@t3_avgAcc, @t3_first_euler_in)
  acc2ang
  math.a2d(@t3_dcm,@t3_first_euler_in)
  d2a

  t3_first_mag[0] := (t3_dcm[0]*t3_first_mag[0] + t3_dcm[1]*t3_first_mag[1] + t3_dcm[2]*t3_first_mag[2])/CMNSCALE
  t3_first_mag[1] := (t3_dcm[3]*t3_first_mag[0] + t3_dcm[4]*t3_first_mag[1] + t3_dcm[5]*t3_first_mag[2])/CMNSCALE
  t3_first_mag[2] := (t3_dcm[6]*t3_first_mag[0] + t3_dcm[7]*t3_first_mag[1] + t3_dcm[8]*t3_first_mag[2])/CMNSCALE

  
  repeat counter from 0 to 8
    t3_first_dcm[counter] := t3_dcm[counter]
    if counter <3
      t3_first_euler_out[counter] := t3_euler[counter] 

PUB acc2ang | x, y, temp

  temp := t3_avgAcc[2] * t3_avgAcc[2]+t3_avgAcc[1] * t3_avgAcc[1]
  x := sqrt(temp)
  y := t3_acc[0] 

  t3_first_euler_in[0] := tr.atan2(x, y)  ' theta
  t3_first_euler_in[1] := tr.atan2(-t3_avgAcc[2], -t3_avgAcc[1])
  t3_first_euler_in[2] := 0
  
PUB sqrt(value)| x, i

  x := value

  repeat i from 0 to 20
    x := (value/x + x) /2

  return x
PUB getSign(value)

  if value >= 0
    result := 1
  else
    result := -1

PUB getOmega |ki 

  'ki := 500
'  ki := 50
  ki := 100
  repeat t3_counter from 0 to 2
    t3_omega[t3_counter] := t3_gyro[t3_counter]*CMNSCALE/131*314/100/180   '10_000 rad/s
    if (t3_omega[t3_counter] < 70 AND t3_omega[t3_counter] > -70)  ' for now eliminate gyro noise
      t3_omega[t3_counter] := 0 
  t3_omega[t3_counter] += t3_I[t3_counter]* ki/10000 /CMNSCALE
    
PUB getEye
  t3_eye[0] := 10000
  t3_eye[1] := 0
  t3_eye[2] := 0
  t3_eye[3] := 0
  t3_eye[4] := 10000
  t3_eye[5] := 0
  t3_eye[6] := 0
  t3_eye[7] := 0
  t3_eye[8] := 10000    


PUB d2a | counter, temp1[9]

  repeat counter from 0 to 8
    temp1[counter] := t3_dcm[counter] * 32768 /CMNSCALE
  
  t3_euler[0] := -tr.asin(temp1[6]*2)           ' q, pitch, theta
  t3_euler[1] := tr.atan2(temp1[8], temp1[7]) ' p, roll, psi  
  t3_euler[2] := tr.atan2(temp1[0], temp1[3]) ' r, yaw, phi



PUB stopDcm
  if t3_dcmCogId
    cogstop(t3_dcmCogId ~ -1)
 
PUB startDcm
  stopDcm
  t3_dcmCogId := cognew(runDcm, @t3_dcmStack) + 1

PUB runDcm
  
  repeat
    if t3_gyroIsUpdated
      t3_prev_dcm := cnt   

      'calculate DCM w/o compensation
      calcDcm
      
      t3_prev_compensation := cnt

      'compensate only when it is available
      if t3_accMagIsUpdated
        calcCompensation
        t3_accMagIsUpdated := 0
       
      d2a


      t3_gyroIsUpdated := 0
      t3_dcmIsUpdated := 1  ' to report to upper level object
      t3_dt_dcm := cnt - t3_prev_dcm
     ' t3_prev_dcm := cnt         uncomment this to measure synched frequency  of DCM w/o compensation
      
PUB calcDcm
  
  dcmStep1
  dcmStep2

PUB calcCompensation

  dcmStep3
  dcmStep4
  dcmStep5
  dcmStep6
  dcmStep7
  dcmStep8


'-----------------------------------------------------------
'-----------------------------------------------------------
'-----------------------------------------------------------
'-----------------------------------------------------------

PUB dcmStep1

  getOmega

  t3_imdt[0] := 0
  t3_imdt[1] := -t3_omega[2]
  t3_imdt[2] := t3_omega[1]
  t3_imdt[3] := t3_omega[2]
  t3_imdt[4] := 0
  t3_imdt[5] := -t3_omega[0]
  t3_imdt[6] := -t3_omega[1]  
  t3_imdt[7] := t3_omega[0]
  t3_imdt[8] := 0
  
  skewOmegaAnomalyChecker

'  repeat t3_counter from 0 to 8
'    t3_matrix_monitor1[t3_counter] := t3_imdt[t3_counter]

  t3_freq_mpu := clkfreq / t3_dt_mpu

   getEye
  repeat t3_counter from 0 to 8
    t3_imdt[t3_counter] := t3_imdt[t3_counter] / t3_freq_mpu
    t3_imdt[t3_counter] := t3_eye[t3_counter] + t3_imdt[t3_counter]

'  repeat t3_counter from 0 to 8
'    t3_matrix_monitor2[t3_counter] := t3_imdt[t3_counter]
    
  t3_imdt2[0] := t3_dcm[0]*t3_imdt[0] + t3_dcm[1]*t3_imdt[3] + t3_dcm[2]*t3_imdt[6]
  t3_imdt2[1] := t3_dcm[0]*t3_imdt[1] + t3_dcm[1]*t3_imdt[4] + t3_dcm[2]*t3_imdt[7]
  t3_imdt2[2] := t3_dcm[0]*t3_imdt[2] + t3_dcm[1]*t3_imdt[5] + t3_dcm[2]*t3_imdt[8] 
  t3_imdt2[3] := t3_dcm[3]*t3_imdt[0] + t3_dcm[4]*t3_imdt[3] + t3_dcm[5]*t3_imdt[6]
  t3_imdt2[4] := t3_dcm[3]*t3_imdt[1] + t3_dcm[4]*t3_imdt[4] + t3_dcm[5]*t3_imdt[7]
  t3_imdt2[5] := t3_dcm[3]*t3_imdt[2] + t3_dcm[4]*t3_imdt[5] + t3_dcm[5]*t3_imdt[8]
  t3_imdt2[6] := t3_dcm[6]*t3_imdt[0] + t3_dcm[7]*t3_imdt[3] + t3_dcm[8]*t3_imdt[6]
  t3_imdt2[7] := t3_dcm[6]*t3_imdt[1] + t3_dcm[7]*t3_imdt[4] + t3_dcm[8]*t3_imdt[7]
  t3_imdt2[8] := t3_dcm[6]*t3_imdt[2] + t3_dcm[7]*t3_imdt[5] + t3_dcm[8]*t3_imdt[8]

  repeat t3_counter from 0 to 8
    t3_imdt2[t3_counter] := t3_imdt2[t3_counter]/CMNSCALE   
    t3_dcm[t3_counter] := t3_imdt2[t3_counter]

 ' repeat t3_counter from 0 to 8
 '   t3_matrix_monitor1[t3_counter] := t3_imdt2[t3_counter]

PUB dcmStep2| il , temp1[9],  col1[3], col2[3], col3[3], err_orth, x_orth[3], y_orth[3], z_orth[3], x_norm[3], y_norm[3], z_norm[3], magnitude[3]

  repeat il from 0 to 2
    col1[il] := t3_dcm[3*il]
    col2[il] := t3_dcm[3*il+1]
    col3[il] := t3_dcm[3*il+2]

  'err_orth = dot(R(:,1)', R(:,2));
  err_orth := (col1[0]*col2[0] + col1[1]*col2[1] + col1[2]*col2[2])/CMNSCALE  
  
  'x_orth = R(:,1) - err_orth/2*R(:,2);
  'y_orth = R(:,2) - err_orth/2*R(:,1);
  repeat il from 0 to 2
    x_orth[il] := col1[il] - col2[il]*err_orth/2/CMNSCALE
    y_orth[il] := col2[il] - col1[il]*err_orth/2/CMNSCALE

  'z_orth = cross(x_orth, y_orth);
  z_orth[0] := (x_orth[1]*y_orth[2] - x_orth[2]*y_orth[1])/CMNSCALE
  z_orth[1] := (x_orth[2]*y_orth[0] - x_orth[0]*y_orth[2])/CMNSCALE
  z_orth[2] := (x_orth[0]*y_orth[1] - x_orth[1]*y_orth[0])/CMNSCALE   

  'x_norm = 0.5*(3-dot(x_orth, x_orth))*x_orth;  
  'y_norm = 0.5*(3-dot(y_orth, y_orth))*y_orth;
  'z_norm = 0.5*(3-dot(z_orth, z_orth))*z_orth;
  magnitude[0] := (x_orth[0]*x_orth[0] + x_orth[1]*x_orth[1] + x_orth[2]*x_orth[2])/CMNSCALE
  magnitude[1] := (y_orth[0]*y_orth[0] + y_orth[1]*y_orth[1] + y_orth[2]*y_orth[2])/CMNSCALE
  magnitude[2] := (z_orth[0]*z_orth[0] + z_orth[1]*z_orth[1] + z_orth[2]*z_orth[2])/CMNSCALE

  repeat il from 0 to 2
    x_norm[il] := (3*x_orth[il] - magnitude[0]*x_orth[il]/CMNSCALE)/2  
    y_norm[il] := (3*y_orth[il] - magnitude[1]*y_orth[il]/CMNSCALE)/2
    z_norm[il] := (3*z_orth[il] - magnitude[2]*z_orth[il]/CMNSCALE)/2


  'R = [x_norm y_norm z_norm];   
  repeat il from 0 to 2
    t3_dcm[il*3] := x_norm[il]
    t3_dcm[il*3+1] := y_norm[il]
    t3_dcm[il*3+2] := z_norm[il]

  'repeat t3_counter from 0 to 8
  '  t3_matrix_monitor2[t3_counter] := t3_dcm[t3_counter] -t3_matrix_monitor1[t3_counter] 
PUB dcmStep3

  t3_acc_body[0] := t3_acc[0] * CMNSCALE /100 * 981 /16384
  t3_acc_body[1] := t3_acc[1] * CMNSCALE /100 * 981 /16384
  t3_acc_body[2] := t3_acc[2] * CMNSCALE /100 * 981 /16384

  t3_acc_earth[0] := (t3_dcm[0]*t3_acc_body[0] + t3_dcm[1]*t3_acc_body[1] + t3_dcm[2]*t3_acc_body[2])/CMNSCALE
  t3_acc_earth[1] := (t3_dcm[3]*t3_acc_body[0] + t3_dcm[4]*t3_acc_body[1] + t3_dcm[5]*t3_acc_body[2])/CMNSCALE
  t3_acc_earth[2] := (t3_dcm[6]*t3_acc_body[0] + t3_dcm[7]*t3_acc_body[1] + t3_dcm[8]*t3_acc_body[2])/CMNSCALE

  t3_mag_earth[0] := (t3_dcm[0]*t3_mag[0] + t3_dcm[1]*t3_mag[1] + t3_dcm[2]*t3_mag[2])/CMNSCALE 
  t3_mag_earth[1] := (t3_dcm[3]*t3_mag[0] + t3_dcm[4]*t3_mag[1] + t3_dcm[5]*t3_mag[2])/CMNSCALE 
  t3_mag_earth[2] := (t3_dcm[6]*t3_mag[0] + t3_dcm[7]*t3_mag[1] + t3_dcm[8]*t3_mag[2])/CMNSCALE 

PUB dcmStep4  | g[3], magSize[2], norm_mag_earth[3], norm_first_mag_earth[3] 

  g[0] := 0
  g[1] := 0
  g[2] := -98100                        
  t3_err_acc_earth[0] := t3_acc_earth[1]*g[2] /CMNSCALE '- acc_earth[2]*g[1])/CMNSCALE
  t3_err_acc_earth[1] := -t3_acc_earth[0]*g[2]/CMNSCALE   'acc_earth[2]*g[0]/CMNSCALE - acc_earth[0]*g[2]/CMNSCALE
  t3_err_acc_earth[2] := 0'acc_earth[0]*g[1]/CMNSCALE - acc_earth[1]*g[0]/CMNSCALE  

  'Mag_earth(i,:) = [Mag_earth(i,1) Mag_earth(i,2) 0] /norm(Mag_earth(i,:)); 
  magSize[0] := math.sqrt(t3_mag_earth[0]*t3_mag_earth[0] + t3_mag_earth[1]*t3_mag_earth[1] )'+mag_earth[2]*mag_earth[2])
  norm_mag_earth[0] := t3_mag_earth[1] * CMNSCALE / magSize[0]
  norm_mag_earth[1] := t3_mag_earth[2] * CMNSCALE / magSize[0]
  norm_mag_earth[2] := 0

  'err_mag_earth = cross(Mag_earth(i,:),  Mag_earth(1,:)); 
  magSize[1] := math.sqrt(t3_first_mag[0]*t3_first_mag[0] +t3_first_mag[1]*t3_first_mag[1]) ' +first_mag_earth[2]*first_mag_earth[2]/CMNSCALE)
  norm_first_mag_earth[0] := t3_first_mag[0] * CMNSCALE / magSize[1]
  norm_first_mag_earth[1] := t3_first_mag[1] * CMNSCALE / magSize[1]
  norm_first_mag_earth[2] := t3_first_mag[2] * CMNSCALE / magSize[1] 

  t3_err_mag_earth[0] := 0'(norm_mag_earth[1]*norm_first_mag_earth[2] - norm_mag_earth[2]*norm_first_mag_earth[1])/CMNSCALE
  t3_err_mag_earth[1] := 0'(norm_mag_earth[2]*norm_first_mag_earth[0] - norm_mag_earth[0]*norm_first_mag_earth[2])/CMNSCALE
  t3_err_mag_earth[2] := (norm_mag_earth[0]*norm_first_mag_earth[1] - norm_mag_earth[1]*norm_first_mag_earth[0])/CMNSCALE 

  'Err_earth(i,:) = err_acc_earth + err_mag_earth;
  t3_err_earth[0] := t3_err_acc_earth[0]
  t3_err_earth[1] := t3_err_acc_earth[1]
  t3_err_earth[2] := t3_err_mag_earth[2] 



PUB dcmStep5 | DCMTrans[9]

  DCMTrans[0] := t3_dcm[0]
  DCMTrans[1] := t3_dcm[3]
  DCMTrans[2] := t3_dcm[6]
  DCMTrans[3] := t3_dcm[1]
  DCMTrans[4] := t3_dcm[4]
  DCMTrans[5] := t3_dcm[7]
  DCMTrans[6] := t3_dcm[2]
  DCMTrans[7] := t3_dcm[5]
  DCMTrans[8] := t3_dcm[8]

  t3_err_body[0] := (DCMTrans[0]*t3_err_earth[0] + DCMTrans[1]*t3_err_earth[1] + DCMTrans[2]*t3_err_earth[2])/CMNSCALE
  t3_err_body[1] := (DCMTrans[3]*t3_err_earth[0] + DCMTrans[4]*t3_err_earth[1] + DCMTrans[5]*t3_err_earth[2])/CMNSCALE
  t3_err_body[2] := (DCMTrans[6]*t3_err_earth[0] + DCMTrans[7]*t3_err_earth[1] + DCMTrans[8]*t3_err_earth[2])/CMNSCALE
  t3_err_body[2] /= 9000
  't3_err_body[2] *= 0 'quick fix of that magnetometer err is too big  

  repeat t3_counter from 0 to 2
   ' t3_matrix_monitor1[t3_counter*3] := t3_acc_body[t3_counter]
   ' t3_matrix_monitor1[t3_counter*3+1] := t3_acc_earth[t3_counter]
   ' t3_matrix_monitor1[t3_counter*3+2] := t3_err_acc_earth[t3_counter]

   ' t3_matrix_monitor2[t3_counter*3] := t3_mag[t3_counter]
   ' t3_matrix_monitor2[t3_counter*3+1] := t3_mag_earth[t3_counter]
   ' t3_matrix_monitor2[t3_counter*3+2] := t3_err_mag_earth[t3_counter]

    t3_matrix_monitor3[t3_counter*3] := t3_err_earth[t3_counter]
    t3_matrix_monitor3[t3_counter*3+1] := t3_err_body[t3_counter]
    't3_matrix_monitor3[t3_counter*3+2] := t3_err_mag_earth[t3_counter]

  't3_err_body[2] :=   0
PUB dcmStep6 | kp

  'kp := 5000 'kp = 0.001
 ' kp := 500
   kp := 1000
  
  'skew(Err_body(i,:))*kp
  t3_imdt[0] := 0
  t3_imdt[1] := -t3_err_body[2] * kp / CMNSCALE
  t3_imdt[2] := t3_err_body[1]  * kp / CMNSCALE
  t3_imdt[3] := t3_err_body[2]  * kp / CMNSCALE
  t3_imdt[4] := 0
  t3_imdt[5] := -t3_err_body[0] * kp / CMNSCALE
  t3_imdt[6] := -t3_err_body[1] * kp / CMNSCALE 
  t3_imdt[7] := t3_err_body[0]  * kp / CMNSCALE
  t3_imdt[8] := 0

'  repeat t3_counter from 0 to 8
'    t3_matrix_monitor1[t3_counter] :=  t3_imdt[t3_counter]


  t3_freq_mpu := clkfreq / t3_dt_mpu

  getEye

  ' (eye(3) + skew(Err_body(i,:))*kp*dt(i))
  repeat t3_counter from 0 to 8
    t3_imdt[t3_counter] := t3_imdt[t3_counter] / t3_freq_mpu
    t3_imdt[t3_counter] := t3_eye[t3_counter] + t3_imdt[t3_counter]

'  repeat t3_counter from 0 to 8
'    t3_matrix_monitor2[t3_counter] :=  t3_imdt[t3_counter]


  'R*(eye(3) + skew(Err_body(i,:))*kp*dt(i))
  t3_imdt2[0] := t3_dcm[0]*t3_imdt[0] + t3_dcm[1]*t3_imdt[3] + t3_dcm[2]*t3_imdt[6]
  t3_imdt2[1] := t3_dcm[0]*t3_imdt[1] + t3_dcm[1]*t3_imdt[4] + t3_dcm[2]*t3_imdt[7]
  t3_imdt2[2] := t3_dcm[0]*t3_imdt[2] + t3_dcm[1]*t3_imdt[5] + t3_dcm[2]*t3_imdt[8] 
  t3_imdt2[3] := t3_dcm[3]*t3_imdt[0] + t3_dcm[4]*t3_imdt[3] + t3_dcm[5]*t3_imdt[6]
  t3_imdt2[4] := t3_dcm[3]*t3_imdt[1] + t3_dcm[4]*t3_imdt[4] + t3_dcm[5]*t3_imdt[7]
  t3_imdt2[5] := t3_dcm[3]*t3_imdt[2] + t3_dcm[4]*t3_imdt[5] + t3_dcm[5]*t3_imdt[8]
  t3_imdt2[6] := t3_dcm[6]*t3_imdt[0] + t3_dcm[7]*t3_imdt[3] + t3_dcm[8]*t3_imdt[6]
  t3_imdt2[7] := t3_dcm[6]*t3_imdt[1] + t3_dcm[7]*t3_imdt[4] + t3_dcm[8]*t3_imdt[7]
  t3_imdt2[8] := t3_dcm[6]*t3_imdt[2] + t3_dcm[7]*t3_imdt[5] + t3_dcm[8]*t3_imdt[8]

  repeat t3_counter from 0 to 8
    t3_imdt2[t3_counter] := t3_imdt2[t3_counter]/CMNSCALE   
    t3_dcm[t3_counter] := t3_imdt2[t3_counter]

PUB dcmStep7

  repeat t3_counter from 0 to 2
    t3_intrmdtI[t3_counter] += t3_err_body[t3_counter]
    t3_I[t3_counter] := -100 #> t3_intrmdtI[t3_counter] <# 100


      
  repeat t3_counter from 0 to 2
    t3_matrix_monitor2[t3_counter*3] := t3_I[t3_counter]

PUB dcmStep8
   
  ' done in 'getOmega

PUB freezResult | local_c

 ' long t3_dcm_d[9], t3_eye_d[9], t3_imdt_d[9]
 ' long t3_omega_d[3] 
  repeat local_c from 0 to 8
    if local_c < 3
      t3_acc_d[local_c] := t3_acc[local_c]
      t3_gyro_d[local_c] := t3_gyro[local_c]    
      t3_mag_d[local_c] := t3_mag[local_c]
      t3_omega_d[local_c] := t3_omega[local_c]
      t3_euler_d[local_c] := t3_euler[local_c]
      t3_avg_acc_d[local_c] := t3_avgAcc[local_c]
      t3_first_mag_d[local_c] := t3_first_mag[local_c]
      t3_first_euler_in_d[local_c] := t3_first_euler_in[local_c]
      t3_first_euler_out_d[local_c] := t3_first_euler_out[local_c]  

    t3_dcm_d[local_c] := t3_dcm[local_c]
    t3_first_dcm_d[local_c] := t3_first_dcm[local_c]
    t3_imdt_d[local_c] := t3_imdt[local_c]
    t3_imdt2_d[local_c] := t3_imdt2[local_c]
    t3_imdt3_d[local_c] := t3_imdt3[local_c]
    
    t3_matrix_monitor1_d[local_c] := t3_matrix_monitor1[local_c]
    t3_matrix_monitor2_d[local_c] := t3_matrix_monitor2[local_c]
    t3_matrix_monitor3_d[local_c] := t3_matrix_monitor3[local_c]  
    
 

PRI printAcc | counter

  fds.str(String("accX = ")) 
  fds.decln(t3_acc_d[0])

  fds.str(String("accY = "))  
  fds.decln(t3_acc_d[1])

  fds.str(String("accZ = ")) 
  fds.decln(t3_acc_d[2])

PRI printDt

  fds.str(String("           MPU time      DCM time     Debug time"))
  fds.newline
  
  fds.str(String("dt         "))
  fds.dec(t3_dt_mpu)
  fds.str(String("       ")) 
  fds.dec(t3_dt_dcm)
  fds.str(String("       "))
  fds.dec(t3_dt_fds)
  fds.newline
  
  fds.str(String("freq(Hz)   "))
  fds.dec(clkfreq/t3_dt_mpu)
  fds.str(String("            ")) 
  fds.dec(clkfreq/t3_dt_dcm)
  fds.str(String("            ")) 
  fds.dec(clkfreq/t3_dt_fds) 

  fds.newline

PRI printOmega

  fds.newline
  fds.strLn(String("omega(10 mili rad/s)"))
  fds.str(String("X: "))
  fds.dec(t3_omega_d[0])
  fds.str(String(" Y: "))
  fds.dec(t3_omega_d[1])
  fds.str(String(" Z: "))
  fds.decLn(t3_omega_d[2])
  fds.newline


PRI printFisrtDCM | iter, digit, counter
  
  fds.str(String("First DCM (rad*10000) : "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(t3_first_dcm_d[iter])  
    counter := 0
    fds.dec(t3_first_dcm_d[iter])
    repeat counter from 0 to (12-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))

PRI printDCM | iter, digit, counter
  
  fds.str(String("DCM (rad*10000) : "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(t3_dcm_d[iter])  
    counter := 0
    fds.dec(t3_dcm_d[iter])
    repeat counter from 0 to (12-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))



PRI printMatrixMonitor | iter, digit, counter
  
  fds.str(String("Monitor1 : "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(t3_matrix_monitor1_d[iter])  
    counter := 0
    fds.dec(t3_matrix_monitor1_d[iter])
    repeat counter from 0 to (12-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))

  fds.newline 
  fds.str(String("Monitor2 : "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(t3_matrix_monitor2_d[iter])  
    counter := 0
    fds.dec(t3_matrix_monitor2_d[iter])
    repeat counter from 0 to (12-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))  
            
  fds.newline 
  fds.str(String("Monitor3: Normalized Error * 10_000, err_earth, err_body, empty "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(t3_matrix_monitor3_d[iter])  
    counter := 0
    fds.dec(t3_matrix_monitor3_d[iter])
    repeat counter from 0 to (12-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))

      
PRI getDigit(input)| ans, flag 

  ans := 0
  
  if input < 0
    input := -input
    flag := 1

  if (input <10)
    ans := 1
  elseif (input <100)
    ans := 2
  elseif (input <1000)
    ans := 3
  elseif (input < 10000)
    ans := 4
  elseif (input < 100000)
    ans := 5
  elseif(input <1000000)
    ans:= 6
  elseif(input <10000000)
    ans:= 7
  elseif(input <100000000)
    ans:= 8    
  elseif(input <1000000000)
    ans:= 9
  else
    ans :=10 

  if flag ==1
    ans += 1

  return ans


PRI printAnomalyMonitor

  fds.strLn(String("Anomaly Monitor Running"))
  fds.str(String("t3_an_skew_omega: "))
  if (t3_an_skew_omega > 0)
    fds.dec(t3_an_skew_omega)
    fds.str(String(" Skew Omega Error Detected."))
  else
    fds.str(String(" Skew Omega is working"))
  fds.newline   
 
PRI printAvgAcc | counter
  fds.strLn(String("First average Accel"))
  fds.str(String("accX = ")) 
  fds.decln(t3_avg_acc_d[0])
  'fds.strln(String(" (10000^-1 m/s)"))

  fds.str(String("accY = "))  
  fds.decln(t3_avg_acc_d[1])
  'fds.strln(String(" (10000^-1 m/s)"))
  
  fds.str(String("accZ = ")) 
  fds.decln(t3_avg_acc_d[2])
  'fds.strln(String(" (10000^-1 m/s)"))

PRI printFirstEulerInput

  fds.strLn(String("First Euler Angles"))

  fds.str(String("pitch = "))
  fds.dec(t3_first_euler_in_d[0])
  fds.strLn(String("  centi degree"))

  
  fds.str(String("roll = "))
  fds.dec(t3_first_euler_in_d[1])
  fds.strLn(String("  centi degree"))

  fds.str(string("yaw = "))
  fds.dec(t3_first_euler_in_d[2])
  fds.strLn(String("  centi degree"))

PRI printFirstEulerOutput

  fds.strLn(String("First Euler Angles"))

  fds.str(String("pitch = "))
  fds.dec(t3_first_euler_out_d[0])
  fds.strLn(String("  centi degree"))

  
  fds.str(String("roll = "))
  fds.dec(t3_first_euler_out_d[1])
  fds.strLn(String("  centi degree"))

  fds.str(string("yaw = "))
  fds.dec(t3_first_euler_out_d[2])
  fds.strLn(String("  centi degree"))


PRI printEuler

  fds.strLn(String("Calculated Euler Angles"))

  fds.str(String("pitch = "))
  fds.dec(t3_euler_d[0])
  fds.strLn(String("  centi degree"))

  
  fds.str(String("roll = "))
  fds.dec(t3_euler_d[1])
  fds.strLn(String("  centi degree"))

  fds.str(string("yaw = "))
  fds.dec(t3_euler_d[2])
  fds.strLn(String("  centi degree"))

PRI printEulerOutput


  fds.strLn(String("Calcualted Euler Angles"))

  fds.str(String("pitch = "))
  fds.dec(t3_euler_d[0])
  fds.strLn(String("  centi degree"))

  
  fds.str(String("roll = "))
  fds.dec(t3_euler_d[1])
  fds.strLn(String("  centi degree"))

  fds.str(string("yaw = "))
  fds.dec(t3_euler_d[2])
  fds.strLn(String("  centi degree"))



PRI printFirstMag

  fds.strLn(String("First Magnetometer "))
  fds.str(String("magX = ")) 
  fds.decln(t3_first_mag_d[0])

  fds.str(String("magY = "))  
  fds.decln(t3_first_mag_d[1])
  
  fds.str(String("magZ = ")) 
  fds.decln(t3_first_mag_d[2])


PRI printCurrentMag



  fds.strLn(String("Current Magnetometer "))
  fds.str(String("magX = ")) 
  fds.decln(t3_mag_d[0])

  fds.str(String("magY = "))  
  fds.decln(t3_mag_d[1])
  
  fds.str(String("magZ = ")) 
  fds.decln(t3_mag_d[2])
  
PUB skewOmegaAnomalyChecker
  t3_an_skew_omega := 0
  if (t3_imdt[0] <> 0)
    t3_an_skew_omega := 1
  if (t3_imdt[1] <> -t3_omega[2])
    t3_an_skew_omega := 1
  if (t3_imdt[2] <> t3_omega[1])
    t3_an_skew_omega := 1
  if (t3_imdt[3] <> t3_omega[2])
    t3_an_skew_omega := 1
  if (t3_imdt[4] <> 0)
    t3_an_skew_omega := 1   
  if (t3_imdt[5] <> -t3_omega[0])
    t3_an_skew_omega := 1
  if (t3_imdt[6] <> -t3_omega[1])
    t3_an_skew_omega := 1
  if (t3_imdt[7] <> t3_omega[0])
    t3_an_skew_omega := 1   

PUB omegaDtAnomalyChecker