CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
                      
VAR
  'motor variables
  long motorPin[6], pulsePtr[6], senM[4]
   
{{
-----------------------------------------------------------------
MOTOR CONTROL REGION                                            |
  Number of cog used : 1                                        |
  Motors             : Brushless DC motor x 6                   |
  Functions:         : initMotor                                |
                       runMotor                                 |
                       insepctPulse                             |
-----------------------------------------------------------------
}}

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

PUB runMotor | check, baseTime, totalElapse, i , array[6]             {{generating pwm for the motor connected to this pin}}              
  
  initMotor  'physical initialization for this motor 

  array[0] := 0
  array[1] := 3
  array[2] := 1
  array[3] := 4
  array[4] := 2
  array[5] := 5
  
  repeat i from 0 to 5
    dira[motorPin[i]] := 1   'set pin direction for this motor 
    long[pulsePtr][i] := 1200         'set default pwm

  repeat
    inspectPulse ' when abnormal pwm comes, udpate all wpm as 1200
    
    totalElapse := 0
    baseTime := cnt    

    repeat i from 0 to 5
      outa[motorPin[array[i]]]:= 1
      waitcnt(cnt + clkfreq/1000000*long[pulsePtr][array[i]])
      outa[motorPin[array[i]]]:= 0
         
    'repeat i from 0 to 5
      totalElapse += long[pulsePtr][i]
       
    waitcnt(baseTime + (clkfreq/1000*25 - clkfreq/1000000*totalElapse))

PRI inspectPulse | i , j

  repeat i from 0 to 5
    if ((long[pulsePtr][i] < 1100) OR (1950 < long[pulsePtr][i]))
      repeat j from 0 to 5
        long[pulsePtr][j] := 1200
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