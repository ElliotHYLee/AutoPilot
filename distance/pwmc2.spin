CON
  _clkmode = xtal1 + pll16x                                               'Standard clock mode * crystal frequency = 80 MHz
  _xinfreq = 5_000_000

VAR
  long  stack[128], ping, gctra, counter, dist
   
OBJ
  uart : "FullDuplexSerial.spin"
  
PUB main

  uart.quickStart
                       
  cognew(report, @stack) 

  repeat
    ping := pulse_in(9)  ' ping in mm

pub pulse_in(pin) | mask

  dira[9] := 0  

  if(outa[9] ==0)
    result := 1
  else
    result := 2

  
PUB report | temp

  repeat
    uart.clear
    uart.dec(ping)
    uart.newline
    temp := ping
    dist := temp'dist*70/100 + temp*30/100
    uart.dec(dist/10)
    uart.str(string("."))
    uart.dec(dist//10)
    uart.strln(string(" cm"))
    waitcnt(cnt + clkfreq/10)