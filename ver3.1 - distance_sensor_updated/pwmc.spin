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
  
  {
  dira[8] := 0
  
  ctra := %01000_000  << 23 + SIGNAL 'Establish mode and APIN (BPIN is ignored)
  frqa := 1
  dira[8]~

  gctra := ctra

  repeat
    if (counter := phsa) ' if pin is high and, obviously, phsa is not 0
      if counter == phsa  ' if pin is low(phsa stopped increasing)
        ping := phsa ~    ' update ping and clear phsa
  }
pub pulse_in(pin) | mask, milimeter

  mask := 1 << pin                                              ' mask for pin

  frqa := 1                                                     ' set for high-resolution measurement

  ctra := (%01000 << 26) | pin                                  ' set ctra to POS detect on pin   
  waitpne(mask, mask, 0)                                        ' wait for pin to be low
  phsa := 0                                                     ' clear accumulator
  waitpeq(mask, mask, 0)                                        ' wait for pin to go high
  waitpne(mask, mask, 0)                                        ' wait for pin to return low

  milimeter := phsa / (clkfreq / 1_000_000)   ' convert ticks to us
  
  return  milimeter                            


  
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