CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 6_250_000

  db = 0
  
OBJ

  debug         : "fullDuplexSerial4port_tier2.spin"
                  
VAR
  'motor variables
  long motorPin[6], pulsePtr[6], stack1[50], stack2[50], base[5], dt[5]

  'debug
  long pulse[6]
   
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
    pulse[i] := 2000

  setMotorPWM(@pulse)
  
  setMotorPins(6,7,8,9,10,11)
  runMotor

  debug.quickStartDebug

  repeat
    debug.clear(db)
    
    debug.str(db,String("1: freq: "))
    debug.decLn(db, clkfreq/dt[0])

    debug.str(db,String("2: freq: "))
    debug.dec(db, clkfreq/dt[1])

    
    waitcnt(cnt + clkfreq/10)
  
  


PUB setMotorPins(pin0, pin1, pin2, pin3, pin4, pin5)  {{ constructor }}

  motorPin[0] := pin0  'set pin number for this motor
  motorPin[1] := pin1
  motorPin[2] := pin2
  motorPin[3] := pin3
  motorPin[4] := pin4
  motorPin[5] := pin5

PUB setMotorPWM(pwmPtr) | i
  repeat i from 0 to 5
    pulsePtr[i] := pwmPtr[i]

PUB runMotor             {{generating pwm for the motor connected to this pin}}              
  
  initMotor  'physical initialization for this motor 

  cognew(runMotor1, @stack1)
  
  cognew(runMotor2, @stack2)
  
PUB runMotor1  | baseTime, totalElapse, i

  repeat i from 0 to 2
    dira[motorPin[i]] := 1   'set pin direction for this motor 
    long[pulsePtr][i] := 1200         'set default pwm

  repeat
    inspectPulse ' when abnormal pwm comes, udpate all wpm as 1200
    totalElapse := 0
    baseTime := cnt    
    repeat i from 0 to 2
      outa[motorPin[i]]:= 1
      waitcnt(cnt + clkfreq/1_000_000*long[pulsePtr][i])
      outa[motorPin[i]]:= 0
      totalElapse += long[pulsePtr][i]
    waitcnt(baseTime + (clkfreq/1000*10 - clkfreq/1_000_000*totalElapse))
    dt[0] := cnt - baseTime

PUB runMotor2  | baseTime, totalElapse, i 

  repeat i from 3 to 5
    dira[motorPin[i]] := 1   'set pin direction for this motor 
    long[pulsePtr][i] := 1200         'set default pwm

  repeat
    inspectPulse ' when abnormal pwm comes, udpate all wpm as 1200
    totalElapse := 0
    baseTime := cnt    
    repeat i from 3 to 5
      outa[motorPin[i]]:= 1
      waitcnt(cnt + clkfreq/1_000_000*long[pulsePtr][i])
      outa[motorPin[i]]:= 0
      totalElapse += long[pulsePtr][i]
    waitcnt(baseTime + (clkfreq/1000*10 - clkfreq/1_000_000*totalElapse))
    dt[1] := cnt - baseTime

    
PRI inspectPulse | i , j

  repeat i from 0 to 5
    if ((long[pulsePtr][i] < 1100) OR (1800 < long[pulsePtr][i])) '(2050 < long[pulsePtr][i]))
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