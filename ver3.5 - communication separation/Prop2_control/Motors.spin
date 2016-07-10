CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 6_250_000

  db = 0
  
OBJ

  debug         : "fullDuplexSerial4port_tier2.spin"
                  
VAR
  'motor variables
  long motorPin[6], pulsePtr[6], stack1[100], stack2[100], base[5], dt[5]

  'lock
  long motorLockPtr
  
  'debug
  long pulse[6], alive[2], targetPulse, stopPulse
   
{{
-----------------------------------------------------------------
MOTOR CONTROL REGION                                            |
  Number of cog used : 2                                        |
  Motors             : Brushless DC motor x 6                   |
  Functions:         : initMotor                                |
                       runMotor                                 |
                       insepctPulse                             |
-----------------------------------------------------------------
}}
PUB main | i

  repeat i from 0 to 5
    pulse[i] := 1100

  
  setMotorPWM(@pulse)
  
  setMotorPins(6,7,8,9,10,11)
  runMotor

  debug.quickStartDebug

  repeat
    debug.clear(db)
    
    debug.str(db,String("1: freq: "))
    debug.decLn(db, clkfreq/dt[0])
    if (alive[0] == 1)
      debug.strLn(db, String("1 is alive"))
      alive[0] := 0
    else
      debug.strLn(db, String("1 is dead"))
      
    debug.newline(db)
    debug.str(db,String("2: freq: "))
    debug.decLn(db, clkfreq/dt[1])
    if (alive[1] == 1)
      debug.strLn(db, String("2 is alive"))
      alive[1] := 0
    else
      debug.strLn(db, String("2 is dead"))

    debug.newline(db)
    repeat i from 0 to 5
      debug.dec(db, i)
      debug.str(db, string(": "))
      debug.decLn(db, long[pulsePtr][i])
      
    
    waitcnt(cnt + clkfreq/10)
  
PUB givemeStopPulse
 return @stopPulse
  
PUB givemeAlive
  return @alive

PUB setMotorLock(val)

  motorLockPtr := val


PUB setMotorPins(pin0, pin1, pin2, pin3, pin4, pin5)  {{ constructor }}

  motorPin[0] := pin0  'set pin number for this motor
  motorPin[1] := pin1
  motorPin[2] := pin2
  motorPin[3] := pin3
  motorPin[4] := pin4
  motorPin[5] := pin5

PUB setMotorPWM(pwmPtr)
  pulsePtr := pwmPtr

PUB runMotor             {{generating pwm for the motor connected to this pin}}              
  
  initMotor  'physical initialization for this motor 

  cognew(runMotor1, @stack1)
  
  cognew(runMotor2, @stack2)
  
PUB runMotor1  | baseTime, dutyBase, totalElapse, i

  repeat i from 0 to 2
    dira[motorPin[i]] := 1   'set pin direction for this motor 
    'long[pulsePtr][i] := targetPulse         'set default pwm

  repeat
    'inspectPulse ' when abnormal pwm comes, udpate all wpm as 1200
    totalElapse := 0
    baseTime := cnt    
    repeat i from 0 to 2
      outa[motorPin[i]]:= 1
      'repeat while (long[motorLockPtr]==1)
      stopPulse := long[pulsePtr][i]
      waitcnt(cnt + clkfreq/1_000_000*long[pulsePtr][i])
      outa[motorPin[i]]:= 0
      totalElapse += long[pulsePtr][i]
    waitcnt(baseTime + (clkfreq/150 - totalElapse))  
    alive[0] := 1
    dt[0] := cnt - baseTime

PUB runMotor2  | baseTime, totalElapse, i 

  repeat i from 3 to 5
    dira[motorPin[i]] := 1   'set pin direction for this motor 
    'long[pulsePtr][i] := targetPulse         'set default pwm

  repeat
    'inspectPulse ' when abnormal pwm comes, udpate all wpm as 1200
    totalElapse := 0
    baseTime := cnt    
    repeat i from 3 to 5
      outa[motorPin[i]]:= 1
      waitcnt(cnt + clkfreq/1_000_000*long[pulsePtr][i])
      outa[motorPin[i]]:= 0
      totalElapse += long[pulsePtr][i]
    waitcnt(baseTime + (clkfreq/150 - totalElapse))
    alive[1] := 1
    dt[1] := cnt - baseTime

    
PRI inspectPulse | i , j

  repeat i from 0 to 2
    if ((long[pulsePtr][i] < 1100) OR (2000 < long[pulsePtr][i])) '(2050 < long[pulsePtr][i]))
      repeat j from 0 to 5
        long[pulsePtr][j] := 1100
      return 0
      
PRI initMotor | i {{initializing the motor connected to this pin}}

 'set pin directions               
  repeat i from 0 to 5
    dira[motorPin[i]] := 1

  repeat 10
    repeat i from 0 to 5
      outa[motorPin[i]]:=1
      waitcnt(cnt + (clkfreq / 1000 ) )
      outa[motorPin[i]]:=0
    waitcnt(cnt + clkfreq / 1000*20)

    