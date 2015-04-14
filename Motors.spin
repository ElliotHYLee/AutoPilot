CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
                      
VAR
  'motor variables
  long motorPin[4], pulsePtr[4], senM[4]
  byte motorIteration 

   
{{
-----------------------------------------------------------------
MOTOR CONTROL REGION                                            |
  Number of cog used : 0                                        |
  Motors             : Brushless DC motor x 4                   |
  Functions:         : initMotor                                |
                       runMotor                                 |
                       insepctPulse                             |
-----------------------------------------------------------------
}}

PUB setMotorPins(pin0, pin1, pin2, pin3)  {{ constructor }}

  motorPin[0] := pin0  'set pin number for this motor
  motorPin[1] := pin1
  motorPin[2] := pin2
  motorPin[3] := pin3

PUB setMotorPWM(pwmPtr) | i
  repeat i from 0 to 3
    pulsePtr[i] := pwmPtr[i]
    i++

PUB setMotorSensitivity(s1, s2, s3, s4)



PUB runMotor | check, baseTime, totalElapse, i                 {{generating pwm for the motor connected to this pin}}              
  
  initMotor  'physical initialization for this motor 
  motorIteration := 0
  repeat while motorIteration<4
    dira[motorPin[motorIteration]] := 1   'set pin direction for this motor 
    long[pulsePtr][motorIteration] := 1200         'set default pwm
    motorIteration++

  senM[2] := 1000
  senM[0] := 1034
  
  repeat
    check := inspectPulse
    if check == 0 'abnormaly
      'usb.str(String("running well"))
       totalElapse:=0
       baseTime := cnt    

       repeat i from 0 to 3
         outa[motorPin[i]]:= 1
         waitcnt(baseTime + clkfreq/1000000*1150)
         outa[motorPin[i]]:= 0

       totalElapse := long[pulsePtr][0] + long[pulsePtr][1] + long[pulsePtr][2] + long[pulsePtr][3]
       waitcnt(baseTime + (clkfreq/1000*20 - clkfreq/1000000*totalElapse))      

    else   ' good to go
    
      'usb.str(String("running well"))
       totalElapse:=0
       baseTime := cnt    

'       repeat i from 0 to 3
       outa[motorPin[0]]:= 1
       waitcnt(baseTime + clkfreq/1000000*long[pulsePtr][0]*senM[0]/senM[2])
       outa[motorPin[0]]:= 0
         
       outa[motorPin[1]]:= 1 
       waitcnt(cnt + clkfreq/1000000*long[pulsePtr][1])
       outa[motorPin[1]]:= 0
       
       outa[motorPin[2]]:= 1
       waitcnt(cnt + clkfreq/1000000*long[pulsePtr][2])
       outa[motorPin[2]]:= 0
        
       outa[motorPin[3]]:= 1
       waitcnt(cnt + clkfreq/1000000*long[pulsePtr][3])
       outa[motorPin[3]]:= 0
       totalElapse := long[pulsePtr][0] + long[pulsePtr][1] + long[pulsePtr][2] + long[pulsePtr][3]
       waitcnt(baseTime + (clkfreq/1000*20 - clkfreq/1000000*totalElapse))

PRI inspectPulse | i
  i:=0
  repeat while i < 4
    if ((long[pulsePtr][i] < 1100) OR (2000 < long[pulsePtr][i]))
      return 0   ' abnormal pwm
    i++
  return 1


PRI initMotor  {{initializing the motor connected to this pin}}
  motorIteration:=0                       'set pin directions               
  repeat while motorIteration<4
    dira[motorPin[motorIteration]] := 1
    long[pulsePtr][motorIteration] :=45
    motorIteration++  
  
  repeat while long[pulsePtr][0] < 150
    motorIteration:=0  
    repeat while motorIteration<4
      outa[motorPin[motorIteration]]:=1
      waitcnt(cnt + (clkfreq / 1000 ) )
      outa[motorPin[motorIteration]]:=0
      long[pulsePtr][motorIteration] ++
      motorIteration++
    waitcnt(cnt + clkfreq / 1000*20)