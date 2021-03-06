CON
        
  _clkmode = xtal1 + pll16x     
  _xinfreq = 6_250_000

  db = 0
  kin = 1
  
VAR

  ' IMU values
  long memTx1_init, A1[16]         
  long memRx1_init, euler[3], distGround, DCM[9], gyro[3]  
        
  ' Comm3
  long memTx3_init, motorPWM[6], throttle, lc[3], ctrlRef_Att[3], ctrlRef_Nav[3], pid_att[22], pid_nav[24], consumedIndicator[20]  ' this line: 83 longs total 
  long memRx3_init, attPidConst[10], attPidOnOff[2], attCtrlRef[4], navPidConst[12], navPidOnOff[2], navCtrlRef[4], motor_input[7], throttle_input[2], sys_mode[2], empty[7]
  long comm3ManagerStack[100]

  'pid controller
  long pidStack[100], pidTick[2], pidTock[2]
  byte attPidIsReset[3]
  long proP, derP, integralP, outputP 
  long proR, derR, integralR, outputR
  long proY, derY, integralY, outputY

  long proL, derL, integralL, outputL
  long proD, derD, integralD, outputD
  long proH, derH, integralH, outputH
  long baseThrottle
            
  'locks
  long motorLock
  
  'debug
  long debuggerStack[100]

  'kinect
  long kinectStack[500]

OBJ

  kinect        : "kinect.spin"              
  debug         : "fullDuplexSerial4port_tier2.spin"
  comm1         : "Serial_mirror_full_duplex.spin"
  comm3         : "Serial_mirror_full_duplex.spin"
  motors        : "Motors.spin"
  pidAtt        : "PIDAtt.spin"
  pidNav        : "PIDNav.spin"

  
PUB Main 

  'cognew(runDebug, @debuggerStack)
  
  cognew(runKinect, @kinectStack)  ' 2 cogs 

  comm1Receiver                    ' 1 cog
  comm3Receiver                    ' 1 cog
  
  setMotor(6,7,8,9,10,11)          ' 2 cogs

  cognew(runPid, @pidStack)        ' 1 cog

  comm3Manager                     ' 1 repeat

  

PUB runDebug | i, alivePtr, spPtr
    
  debug.quickStartDebug 
  repeat
    debug.clear(db)
    repeat i from 0 to 5
      debug.dec(db,memTx1_init[i])
      debug.str(db,String("     "))
      debug.dec(db,memRx1_init[i])
      debug.newline(db)     
                          
    'debug.newline(db)      
    
    repeat i from 0 to 50
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

    alivePtr := motors.givemeAlive       
    
    if (long[alivePtr][0] == 1)
      debug.strLn(db, String("1 is alive"))
      long[alivePtr][0] := 0
    else                                                            
      debug.strLn(db, String("1 is dead"))
    debug.str(db, string("sp: "))
    spPtr := motors.givemeStopPulse
    debug.decLn(db,long[spPtr]) 
      
    if (long[alivePtr][1] == 1)
      debug.strLn(db, String("2 is alive"))
      long[alivePtr][1] := 0
    else
      debug.strLn(db, String("2 is dead"))
                    
    waitcnt(cnt + clkfreq/10)     

PUB runKinect

  kinect.setLocalCoord(@lc)
  kinect.init
  kinect.runKinect
  
PUB setUpAttPidConst

  pid_att[1]  := 500  'att pid x kp , x= pitch
  pid_att[2]  := 10_000  'att pid x kd
  pid_att[3]  := 60_000  'att pid x ki

  pid_att[8]  := 600  'att pid y kp, y = roll     kp = 0.03
  pid_att[9]  := 10_000  'att pid y kd            kd = 1
  pid_att[10] := 60_000  'att pid y ki           ki = 0.1
                                             
  pid_att[15] := 600     '600  'att pid z kp, z = yaw
  pid_att[16] := 400     '200_000  'att pid z kd
  pid_att[17] := 60_000  '10_000'30000  'att pid z ki
  
PUB setUpNavPidConst

  pid_nav[1]  := 50    '0.5  
  pid_nav[2]  := 50  
  pid_nav[3]  := 1  

  pid_nav[8]  := 100
  pid_nav[9]  := 30  
  pid_nav[10] := 1  
                                             
  pid_nav[15] := 4              'kp up   
  pid_nav[16] := 1              'kp down
  pid_nav[17] := 0              'kd up
  pid_nav[18] := 10              'kd down 
  pid_nav[19] := 0

PUB runPid

  setUpNavPidConst
  setUpAttPidConst
  
  ctrlRef_Att[0] := 0
  ctrlRef_Att[1] := 0
  ctrlRef_Att[2] := -9925 'euler[2]
  
  pidTick[1] := 0
  repeat
    pidTick[0] := cnt 

    if (pidTick[1] => 7)  ' once a five iteration= 30Hz 
      navPid
      pidTick[1] := 0
    else
      resetNavPid
      pidTick[1]++
      
    attPid
     
    repeat while (cnt =< pidTick[0] + clkfreq/150)

    
PUB resetNavPid

  ctrlRef_Att[0] := ctrlRef_Att[0]*10/10   
  ctrlRef_Att[1] := ctrlRef_Att[1]*10/10
  
PUB navPID

  if (pid_nav[0] & %100) 'longitudinal pid 
    xPosPID(pid_nav[1], pid_nav[2], pid_nav[3], 1700)'ctrlRef_Nav[0])  
      
  if (pid_nav[0] & %010) 'directional pid
    yPosPID(pid_nav[8], pid_nav[9], pid_nav[10], ctrlRef_Nav[1])
  else
    
    
  if (pid_nav[0] & %001) 'height pid
    zPosPID( pid_nav[15], pid_nav[16], pid_nav[17],pid_nav[18], pid_nav[19], ctrlRef_nav[2])
  else
    zReset
    

PUB xPosPID(Kp, Kd, Ki, targetPos)

  pidNav.xCalc(Kp, Kd, Ki, lc[0], targetPos, @proL, @derL, @integralL, @outputL)

  ctrlRef_Att[0] := outputL
  
  pid_nav[4] := proL
  pid_nav[5] := derL
  pid_nav[6] := integralL                 
  pid_nav[7] := outputL

PUB xReset

  ctrlRef_Att[0] := 0
  pidNav.xReset

PUB yPosPID(Kp, Kd, Ki, targetPos)

  pidNav.yCalc(Kp, Kd, Ki, lc[1], targetPos, @proD, @derD, @integralD, @outputD)

  ctrlRef_Att[1] := outputD
  
  pid_nav[11] := proD
  pid_nav[12] := derD
  pid_nav[13] := integralD                 
  pid_nav[14] := outputD

PUB yReset
  ctrlRef_Att[1] := 0
  pidNav.yReset

PUB zPosPID(Kp_asc, Kp_des, Kd_asc, Kd_des, Ki, targetPos)

  pidNav.zCalc(Kp_asc, Kp_des, Kd_asc, Kd_des, Ki, lc[2], targetPos, @proH, @derH, @integralH, @outputH)

  throttle := 1450 #> baseThrottle + outputH <# 1700

  pid_nav[20] := proH
  pid_nav[21] := derH
  pid_nav[22] := integralH                 
  pid_nav[23] := outputH 

PUB zReset

  throttle := baseThrottle
  pidNav.zReset

PUB attPID  | thrustBound[2]

  thrustBound[0] := 1200
  thrustBound[1] := 2000
  
  if ((pid_att[0] & %100)) 'pid pitch 
    pitchPID(pid_att[1], pid_att[2], pid_att[3], ctrlRef_Att[0])
    attPidIsReset[0] := false
  else
    if (!attPidIsReset[0])
      resetPitch

  if ((pid_att[0] & %010)) 'pid roll
    rollPID(pid_att[8], pid_att[9], pid_att[10], ctrlRef_Att[1])
    attPidIsReset[1] := false
  else
    if (!attPidIsReset[1])
      resetRoll
    
  if ((pid_att[0] & %001)) 'pid yaw
    yawPID(pid_att[15], pid_att[16], pid_att[17], ctrlRef_Att[2])
    attPidIsReset[2] := false   
  else
    if (!attPidIsReset[2])
      resetYaw

      
  if ( (pid_att[0] & %100) OR (pid_att[0] & %010) OR (pid_att[0] & %001) )
    motorPWM[0] := thrustBound[0] #> throttle - pid_att[14]*86/100 + pid_att[7]  - pid_att[21]  <# thrustBound[1]
    motorPWM[1] := thrustBound[0] #> throttle + pid_att[14]*86/100 + pid_att[7]  + pid_att[21]  <# thrustBound[1]  
    motorPWM[2] := thrustBound[0] #> throttle + pid_att[14]*86/100                              <# thrustBound[1]  
    motorPWM[3] := thrustBound[0] #> throttle + pid_att[14]*86/100 - pid_att[7]  - pid_att[21]  <# thrustBound[1]  
    motorPWM[4] := thrustBound[0] #> throttle - pid_att[14]*86/100 - pid_att[7]  + pid_att[21]  <# thrustBound[1]  
    motorPWM[5] := thrustBound[0] #> throttle - pid_att[14]*86/100                              <# thrustBound[1]  

      
PUB pitchPID(Kp, Kd, Ki, targetTheta) 

  pidAtt.pitch(Kp, Kd, Ki, euler[0], targetTheta, gyro[1], @proP, @derP, @integralP, @outputP)
  
  pid_att[4] := proP
  pid_att[5] := derP
  pid_att[6] := integralP                 
  pid_att[7] := outputP 

PUB resetPitch
  pid_att[4] := 0
  pid_att[5] := 0
  pid_att[6] := 0                 
  pid_att[7] := 0
  pidatt.resetPitch  
  attPidIsReset[0] := true

  
PUB rollPID(Kp, Kd, Ki, targetPhi)

  pidAtt.roll(Kp, Kd, Ki, euler[1], targetPhi, gyro[0], @proR, @derR, @integralR, @outputR)      

  pid_att[11] := proR
  pid_att[12] := derR
  pid_att[13] := integralR
  pid_att[14] := outputR
  
PUB resetRoll
  pid_att[11] := 0
  pid_att[12] := 0
  pid_att[13] := 0
  pid_att[14] := 0
  pidatt.resetRoll  
  attPidIsReset[1] := true  

PUB yawPID(Kp, Kd, Ki, targetPsi)

  pidAtt.yaw(Kp, Kd, Ki, euler[2], targetPsi, gyro[2], @proY, @derY, @integralY, @outputY)
  pid_att[18] := proY
  pid_att[19] := derY
  pid_att[20] := integralY
  pid_att[21] := outputY 
  
PUB resetYaw
  pid_att[18] := 0
  pid_att[19] := 0
  pid_att[20] := 0
  pid_att[21] := 0
  pidatt.resetYaw
  attPidIsReset[2] := true     
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
  baseThrottle := throttle  
  repeat i from 0 to 5
    motorPWM[i] := 1200
  motors.setMotorPWM(@motorPWM)
  motors.setMotorLock(@motorLock)
  runMotor
  
PRI runMotor
  motors.runMotor  ' has 2 cogs in it


PUB comm3Manager | i

  repeat
    updateSystemMode
    
    if (throttle_input[0] ==1 )
      setBasethrottle(throttle_input[1])
      consumedIndicator[7] := 1
    else
      consumedIndicator[7] := 0

    if (motor_input[0] == 1)
      motorLock := 1
      repeat i from 0 to 5
        motorPWM[i] := motor_input[i+1]
      motorLock := 0
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
      setBaseThrottle(1100)
      repeat i from 0 to 5
        motorPWM[i] := 1100
      attPidOffAll
     
    if (sys_mode[1] == 1)  'prepare mode
      setBaseThrottle(1200)
      repeat i from 0 to 5  'delete this later
        motorPWM[i] := 1200
      attPidOnAll
  else
    consumedIndicator[8] := 0

PUB setBaseThrottle(val)
  throttle := val
  baseThrottle := throttle  

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