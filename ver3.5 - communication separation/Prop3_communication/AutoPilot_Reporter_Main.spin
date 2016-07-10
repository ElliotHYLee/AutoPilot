CON             
  _clkmode = xtal1 + pll16x     
  _xinfreq = 6_250_000

  db = 0
  xb = 1
  
VAR

  ' comm line 3
  ' for all arrays in tx3, first element is header; 1 indicate immediate update for receiver, otherwise, 0
  long memTx3_init, attPidConst[10], attPidOnOff[2], attPidCtrlRef[4], navPidConst[12], navPidOnOff[2], navPidCtrlRef[4], motor_input[7], throttle_input[2], sys_mode[2], empty[7]  ' this line: 53 longs total
  long memRx3_init, motorPWM[6], throttle, lc[3], ctrlRef_Att[3], ctrlRef_Nav[3], pid_att[22], pid_nav[24], consumedIndicator[20]  ' this line: 83 longs total 
  long comm3ManagerStack[200]
  
  ' comm line 2
  long memTx2_init, emptyBuffer[14]
  long memRx2_init, euler[3], gyro[3], acc[3], mag[3], distGround, empty2  
   
  'to GCS
  long commStack[200]

  'debug
  long debuggerStack[100]

OBJ

  xbee          : "fullDuplexSerial4port_tier3.spin"
  debug         : "fullDuplexSerial4port_tier2.spin"
  comm3         : "Serial_mirror_full_duplex.spin"
  comm2         : "Serial_mirror_full_duplex.spin"
  
PUB Main | i

  memTx2_init := cnt
  repeat i from 0 to 14
    memTx2_init[i+1] := i*10

  cognew(runDebug, @debuggerStack)
  comm2Receiver     ' 1 cog
  comm3Receiver     ' 1 cog
  newCommunication  ' 2 cogs

  cognew(comm3Manager, @comm3ManagerStack[100])
  
  
PUB runDebug | i

  debug.quickStartDebug 
  repeat
    debug.clear(db)
    'debug.newline(db)
   
    repeat i from 0 to 83
      if i==39
        debug.strLn(db, string("          -----pid_nav"))
      if i==17
        debug.strLn(db, string("          -----pid_att"))
      if i==11
        debug.strLn(db, string("          -----ctrlRef_att"))
      if i==14
        debug.strLn(db, string("          -----ctrlRef_nav"))
        
      debug.dec(db,i) 
      if (i < 53)
        debug.str(db, string(": "))
        debug.dec(db, memTx3_init[i])

      
        
      if i => 63
        debug.str(db,String("cf"))
        debug.dec(db, i-63)
      debug.str(db,String("     "))
      debug.dec(db, memRx3_init[i])      
      debug.newline(db)
                      
    {debug.newline(db)
    repeat i from 0 to 14
      debug.dec(db, memTx2_init[i])
      debug.str(db,String("     "))
      debug.dec(db, memRx2_init[i])      
      debug.newline(db)
                          } 
    debug.newline(db)       
    debug.dec(db, xbee.getInfoChoice )
                                            

    waitcnt(cnt + clkfreq/3)



'===================================================================================================
'===================== Comm3 PART from controller ==================================================
'===================================================================================================
PRI comm3Receiver
  memTx3_init := cnt
  comm3.RX_start(@memTx3_init,@memRx3_init, 53,83, 2, 3 , 4,-1)  ' has 1 cog in it
  
'===================================================================================================
'===================== Comm2 PART from sensor ======================================================
'===================================================================================================
PRI comm2Receiver
  memTx2_init := cnt
  comm2.RX_start(@memTx2_init,@memRx2_init,15,15, 5, 6 , 7,-1)  ' has 1 cog in it

'===================================================================================================
'===================== COMMUNICATION PART ==================================================================
'===================================================================================================
PUB newCommunication

  xbee.initialize
    
  ' state vars
  xbee.setLocalCoordinate(@lc)
  xbee.setAttPtr(@euler, @gyro, @acc, @mag, @distGround)

  ' actuator vars
  xbee.setMotPtr(@motorPWM)
  xbee.setThrottle(@throttle)

  ' att_pid vars
  xbee.setTargetAttitude(@ctrlRef_Att)
  xbee.setAttPid(@pid_att)

  ' nav_pid vars
  xbee.setTargetLocalCoordinate(@ctrlRef_Nav)   
  xbee.setNavPid(@pid_nav)

  ' command inputs
  xbee.setCmdInput(@attPidConst, @attPidOnOff, @attPidCtrlRef,@navPidConst, @navPidOnOff, @navPidCtrlRef, @motor_input, @throttle_input, @sys_mode)


  startCommunication

PRI startCommunication  
  cognew(runCommunication, @commStack)
  
PRI runCommunication
  xbee.communicate

'===================================================================================================
'===================== COMM3 Manager PART ==================================================================
'===================================================================================================

PUB comm3Manager | i

  repeat

    if (consumedIndicator[7] == 1)
      throttle_input[0] := 0

    if (consumedIndicator[6] == 1)
      motor_input[0] := 0

    if (consumedIndicator[8] == 1)
       sys_mode[0] := 0

    if (consumedIndicator[1] ==1)
       attPidOnOff[0] := 0
       xbee.setCmdAck(1)

    if (consumedIndicator[0] ==1)
       attPidConst[0] := 0
       xbee.setCmdAck(2)    

    if (consumedIndicator[4] ==1)
       navPidOnOff[0] := 0
       xbee.setCmdAck(3)
       
    if (consumedIndicator[3] ==1)
       navPidConst[0] := 0
       xbee.setCmdAck(3)

    if (consumedIndicator[2] ==1)
       attPidCtrlRef[0] := 0



       
