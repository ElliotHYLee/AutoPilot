CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
                      
VAR
  'motor variables
  long motorPin[6], pulsePtr[6], senM[4],  defaultPWM[6]       
  byte motorIteration

     
  long stack[128]
  long x ,y 

OBJ

  usb : "parallaxserialterminal.spin"
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

PUB main    | i
  usb.init(31, 30, 0, 115200)     
  
  setMotorPins(2,3,4,5,6,7)
  

  cognew(runMotor, @stack)

  repeat i from 0 to 5
    defaultPWM[i] := 1600
  setMotorPWM(@defaultPWM)    
  repeat
    usb.str(String("pause: "))
    usb.decLn(x)
    usb.str(String("totalEla: "))
    usb.decLn(y)
    usb.decLn(Long[pulsePtr][0])

      
PRI sendMotorMsg | i

  repeat i from 0 to 5
    'motor write
    usb.str(String("[m"))
    usb.Dec(i+1)
    usb.Dec(long[pulsePtr][i])
    usb.str(String("]"))


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
    i++

PUB setThrottle(throttlePtr)



PUB setMotorSensitivity(s1, s2, s3, s4)



PUB runMotor | check, baseTime, totalElapse, i                 {{generating pwm for the motor connected to this pin}}              
  
  initMotor  'physical initialization for this motor 
  motorIteration := 0
  repeat while motorIteration < 6
    dira[motorPin[motorIteration]] := 1   'set pin direction for this motor 
    pulsePtr[motorIteration] := @defaultPWM         'set default pwm
    motorIteration++

  repeat
    check := inspectPulse
    usb.str(String("running"))
    if check == 0 'abnormaly
      usb.str(String("running wierd"))
      'usb.str(String("running well"))
       totalElapse:=0
       baseTime := cnt    

       repeat i from 0 to 5
         outa[motorPin[i]]:= 1
         waitcnt(baseTime + clkfreq/1000000*1150)
         outa[motorPin[i]]:= 0

       repeat i from 0 to 5
         totalElapse += long[pulsePtr][i]
       waitcnt(baseTime + (clkfreq/1000*20 - clkfreq/1000000*totalElapse))      

    else   ' good to go
    
       usb.str(String("running well"))
       totalElapse:=0
       baseTime := cnt    

       repeat i from 0 to 5
       outa[motorPin[0]]:= 1
       waitcnt(baseTime + clkfreq/1000000*long[pulsePtr][i])
       outa[motorPin[0]]:= 0

       repeat i from 0 to 5
         totalElapse += long[pulsePtr][i]
       waitcnt(baseTime + (clkfreq/1000*20 - clkfreq/1000000*totalElapse))

       x := (clkfreq/1000*20 - clkfreq/1000000*totalElapse)
       y := totalElapse
       
PRI inspectPulse | i
  i:=0
  repeat while i < 6
    if ((long[pulsePtr][i] < 1100) OR (2500 < long[pulsePtr][i]))
      return 0   ' abnormal pwm
    i++
  return 1


PRI initMotor  {{initializing the motor connected to this pin}}
  motorIteration:=0                       'set pin directions               
  repeat while motorIteration < 6
    dira[motorPin[motorIteration]] := 1
    long[pulsePtr][motorIteration] := 45
    motorIteration++  
  
  repeat while long[pulsePtr][0] < 100
    motorIteration:=0  
    repeat while motorIteration < 6
      outa[motorPin[motorIteration]]:=1
      waitcnt(cnt + (clkfreq / 1000 ) )
      outa[motorPin[motorIteration]]:=0
      long[pulsePtr][motorIteration] ++
      motorIteration++
    waitcnt(cnt + clkfreq / 1000*20)