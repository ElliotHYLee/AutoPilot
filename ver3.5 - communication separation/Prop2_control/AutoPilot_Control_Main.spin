CON
        
  _clkmode = xtal1 + pll16x     
  _xinfreq = 6_250_000

  db = 0

VAR

  ' IMU values
  long memTx1_init, A1[16] 
  long memRx1_init, euler[3], distGround, DCM[9], gyro[3]  

  ' Comm3
  long memTx3_init, motorPWM[6], throttle, lc[3], ctrlRef_Att[3], ctrlRef_Nav[3], pid_att[22], pid_nav[24], consumedIndicator[20]  ' this line: 83 longs total 
  long memRx3_init, attPidConst[10], attPidOnOff[2], attCtrlRef[4], navPidConst[12], navPidOnOff[2], navCtrlRef[4], motor_input[7], throttle_input[2], sys_mode[2], empty[7]
  long comm3ManagerStack[100]

  'pid controller
  long pidStack[100], pidTick[2], pidTock[2], attGlobalIntegral[3]
   
  'debug
  long debuggerStack[100]

OBJ

  kinect        : "fullDuplexSerial4port_tier2.spin"              
  debug         : "fullDuplexSerial4port_tier2.spin"
  comm1         : "Serial_mirror_full_duplex.spin"
  comm3         : "Serial_mirror_full_duplex.spin"
  motors        : "Motors.spin"
  
  
PUB Main 

  cognew(runDebug, @debuggerStack)

  comm1Receiver                ' 1 cog
  comm3Receiver                ' 1 cog
  
  setMotor(6,7,8,9,10,11)      ' 2 cogs




  cognew(runPid, @pidStack)

  
  comm3Manager                 ' 1 cog

PUB runDebug | i

  debug.quickStartDebug 
  repeat
    debug.clear(db)
    {repeat i from 0 to 16
      debug.dec(db,memTx1_init[i])
      debug.str(db,String("     "))
      debug.dec(db,memRx1_init[i])
      debug.newline(db)     }
                          
    'debug.newline(db)
   
    repeat i from 0 to 70
      if i => 51 AND i =< 60
      else
        if i < 51
          debug.dec(db, memRx3_init[i])
        if i => 61
            debug.str(db,String("cf"))
            debug.dec(db,i-61)
        debug.str(db,String("     "))
        debug.dec(db, memTx3_init[i])      
        debug.newline(db)               
    waitcnt(cnt + clkfreq/10)     


PUB setUpPidConst

  pid_att[1] := 1  'att pid x kp , x= pitch
  pid_att[2] := 2  'att pid x kd
  pid_att[3] := 3  'att pid x ki

  pid_att[8] := 11  'att pid y kp, y = roll
  pid_att[9] := 22  'att pid y kd
  pid_att[10] := 33  'att pid y ki

  pid_att[15] := 111  'att pid z kp, z = yaw
  pid_att[16] := 222  'att pid z kd
  pid_att[17] := 333  'att pid z ki
  

PUB runPid 

  setUpPidConst
  ctrlRef_Att[0] := 0
  ctrlRef_Att[1] := 0
  ctrlRef_Att[2] := euler[2]
  
  pidTick[1] := 0
  repeat
    pidTick[0] := cnt 

    if (pidTick[1] => 5)  ' once a five iteration= 30Hz 
      navPid
      pidTick[1] := 0
    else
      pidTick[1]++
      
    attPid
     
    repeat while (cnt =< pidTick[0] + clkfreq/150)

PUB navPID
'  if ((pid_nav[0] & %100)==1) 'longitudinal pid 

      
'  if ((pid_nav[0] & %010)==1) 'directional pid

    
'  if ((pid_nav[0] & %001)==1) 'height pid
   
  
  

    
PUB attPID
  if ((pid_att[0] & %100)) 'pid pitch 
    pitchPID(pid_att[1], pid_att[2], pid_att[3], attCtrlRef[0])

     
  if ((pid_att[0] & %010)) 'pid roll
    rollPID(pid_att[8], pid_att[9], pid_att[10], attCtrlRef[1])
    
  if ((pid_att[0] & %001)) 'pid yaw
    yawPID(pid_att[15], pid_att[16], pid_att[17], attCtrlRef[2])
      
PUB pitchPID(Kp, Kd, Ki, target)| error, pro, der, integral, output

  error := 4
  pro := 3
  der := 2
  integral := 1
  output := pro + der + integral
  
  pid_att[4] := pro
  pid_att[5] := der
  pid_att[6] := integral
  pid_att[7] := output 


PUB rollPID(Kp, Kd, Ki, target)| error, pro, der, integral, output
  error := 44
  pro := 33
  der := 22
  integral := 11

  pid_att[11] := pro
  pid_att[12] := der
  pid_att[13] := integral
  pid_att[14] := output 

PUB yawPID(Kp, Kd, Ki, target)| error, pro, der, integral, output
  error := 444
  pro := 333
  der := 222
  integral := 111
    
  pid_att[18] := pro
  pid_att[19] := der
  pid_att[20] := integral
  pid_att[21] := output 
  
    
'===================================================================================================
'===================== Comm1 PART from sensors =====================================================
'===================================================================================================
PRI comm1Receiver
  memTx1_init := cnt
  comm1.RX_start(@memTx1_init,@memRx1_init,17,17,0,1,2,-1)  ' has 1 cog in it
  
'===================================================================================================
'===================== Comm2 PART to reporter ======================================================
'===================================================================================================
PRI comm3Receiver
  memTx3_init := cnt
  comm3.TX_start(@memTx3_init,@memRx3_init,83,53, 3, 4, 5,-1)  ' has 1 cog in it

'===================================================================================================
'===================== MOTOR PART ==================================================================
'===================================================================================================
PRI setMotor(pin0, pin1, pin2, pin3, pin4, pin5) | i 

  motors.setMotorPins(pin0, pin1, pin2, pin3, pin4, pin5)
  throttle := 1100
  repeat i from 0 to 5
    motorPWM[i] := 1200
  motors.setMotorPWM(@motorPWM)
  runMotor
  
PRI runMotor
  motors.runMotor  ' has 2 cogs in it


PUB comm3Manager | i

  repeat
    updateSystemMode
    
    if (throttle_input[0] ==1 )
      throttle := throttle_input[1]
      consumedIndicator[7] := 1
    else
      consumedIndicator[7] := 0

    if (motor_input[0] == 1)
      repeat i from 0 to 5
        motorPWM[i] := motor_input[i+1]
      consumedIndicator[6] := 1
    else
      consumedIndicator[6] := 0

    if (attPidOnOff[0] ==1)
      pid_att[0] := attPidOnOff[1]
      consumedIndicator[1] := 1
    else
      consumedIndicator[1] := 0 

    if (attPidConst[0] == 1)
      updatePIDAttConst
      consumedIndicator[0] := 1
    else
      consumedIndicator[0] := 0

    if (navPidOnOff[0] ==1)
      pid_nav[0] := navPidOnOff[1] 
      consumedIndicator[4] := 1
    else
      consumedIndicator[4] := 0

    if (navPidConst[0] == 1)
      updatePIDNavConst
      consumedIndicator[3] := 1
    else
      consumedIndicator[3] := 0 
     
    if (attCtrlRef[0] ==1)
      updateCtrlRef
      consumedIndicator[2] := 1
    else
      consumedIndicator[2] := 0


PUB updateCtrlRef

  ctrlRef_Att[0] := attCtrlRef[1]
  ctrlRef_Att[1] := attCtrlRef[2]
  ctrlRef_Att[2] := attCtrlRef[3] 
      
PUB updatePIDNavConst

  pid_nav[1] := navPidConst[1]  'att pid x kp , x= pitch
  pid_nav[2] := navPidConst[2]  'att pid x kd
  pid_nav[3] := navPidConst[3]  'att pid x ki

  pid_nav[8] := navPidConst[4]  'att pid y kp, y = roll
  pid_nav[9] := navPidConst[5]  'att pid y kd
  pid_nav[10] := navPidConst[6]  'att pid y ki

  pid_nav[15] := navPidConst[7]  'att pid z kp, z = yaw
  pid_nav[16] := navPidConst[8]  'att pid z kd
  pid_nav[17] := navPidConst[9]  'att pid z ki
  pid_nav[18] := navPidConst[10]  'att pid z kd
  pid_nav[19] := navPidConst[11]  'att pid z ki

  
PUB updatePIDAttConst

  pid_att[1] := attPidConst[1]  'att pid x kp , x= pitch
  pid_att[2] := attPidConst[2]  'att pid x kd
  pid_att[3] := attPidConst[3]  'att pid x ki

  pid_att[8] := attPidConst[4]  'att pid y kp, y = roll
  pid_att[9] := attPidConst[5]  'att pid y kd
  pid_att[10] := attPidConst[6]  'att pid y ki

  pid_att[15] := attPidConst[7]  'att pid z kp, z = yaw
  pid_att[16] := attPidConst[8]  'att pid z kd
  pid_att[17] := attPidConst[9]  'att pid z ki
 
PUB updateSystemMode | i

  if (sys_mode[0] ==1)
    consumedIndicator[8] := 1 

    if (sys_mode[1] == 0)   'idle mode
      throttle := 1100
      repeat i from 0 to 5
        motorPWM[i] := 1100
      attPidOffAll
     
    if (sys_mode[1] == 1)  'prepare mode
      throttle := 1200
      repeat i from 0 to 5  'delete this later
        motorPWM[i] := 1200
      attPidOnAll
  else
    consumedIndicator[8] := 0

PUB attPidOnAll
  attPidPitchOnOff(1)
  attPidRollOnOff(1)
  attPidYawOnOff(1)
  
PUB attPidOffAll
  attPidPitchOnOff(0)
  attPidRollOnOff(0)
  attPidYawOnOff(0)

PUB attPidPitchOnOff(val)
  if (val==1)
    attPidOnOffSwitch(%100, 1)    'why not used val? for readability
  else
    attPidOnOffSwitch(%011, 0)

PUB attPidRollOnOff(val)
  if (val==1)
    attPidOnOffSwitch(%010, 1)
  else
    attPidOnOffSwitch(%101, 0)

PUB attPidYawOnOff(val)
  if (val==1)
    attPidOnOffSwitch(%001, 1)
  else
    attPidOnOffSwitch(%110, 0)
  
PUB attPidOnOffSwitch(binVal, OnOff)
  if (OnOff ==1)
    pid_att[0] := binVal | pid_att[0]
  else
    pid_att[0] := binVal & pid_att[0] 

