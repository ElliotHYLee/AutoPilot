CON
  _clkmode = xtal1 + pll16x                                               'Standard clock mode * crystal frequency = 80 MHz
  _xinfreq = 5_000_000



  LRF_TX        = 6             ' Serial output to LRF (connects to SIN)
  LRF_RX        = 7             ' Serial input from LRF (connects to SOUT)
 

VAR
  long  stack[128], ping, gctra, counter, dist

  long range                    ' Distance from LRF module to target object (in millimeters)
    
OBJ
 
  serial    : "Extended_FDSerial"                          ' Extended Full Duplex Serial by Martin Hebel (http://obex.parallax.com/objects/31/)
  
  uart : "FullDuplexSerial.spin"
  
PUB main

  uart.quickStart

  serial.Start(LRF_RX, LRF_TX, %0000, 115_200)          ' Start serial port, normal mode, 115.2kbps
  uart.Str(String("Waiting for LRF module..."))
  waitcnt(clkfreq << 1 + cnt)                           ' Delay for 2 seconds to let the LRF module start-up
  serial.Tx("U")                                        ' Send character
  repeat until serial.RxCheck == ":"                    ' When the LRF has initialized and is ready to go, it will send a single ':' character, so wait here until we receive it
  uart.Str(String("Ready!"))             ' Ready to go!
  
                   
  cognew(report, @stack) 

  repeat
    ping := pulse_in(8)

    
    serial.Tx("R")                                      ' Send command
    repeat until serial.RxCheck == "D"                  ' Wait for the header to be sent...
    repeat until serial.RxCheck == "="                  '
    repeat until serial.RxCheck == " "                  '              
    range := serial.RxDec

          
pub pulse_in(pin) | mask

  mask := 1 << pin                                              ' mask for pin

  frqa := 1                                                     ' set for high-resolution measurement

  ctra := (%01000 << 26) | pin                                  ' set ctra to POS detect on pin   
  waitpne(mask, mask, 0)                                        ' wait for pin to be low
  phsa := 0                                                     ' clear accumulator
  waitpeq(mask, mask, 0)                                        ' wait for pin to go high
  waitpne(mask, mask, 0)                                        ' wait for pin to return low

  return phsa / (clkfreq / 1_000_000)                             ' convert ticks to us


  
PUB report | temp

  repeat
    uart.clear
    temp := ping
    dist := temp'dist*70/100 + temp*30/100
    uart.Str(String("USonic Dist = "))  
    uart.dec(dist/10)
    uart.str(string("."))
    uart.dec(dist//10)
    uart.strln(string(" cm"))

    uart.Str(String("Laser Dist = "))
    uart.Dec(range / 10)
    uart.str(String("."))
    uart.Dec(range // 10)
    uart.Strln(String(" cm"))    

    uart.Str(String("Difference = "))
    temp := dist - range
    uart.Dec(temp / 10)
    uart.str(String("."))
    uart.Dec(temp // 10)
    uart.Strln(String(" cm"))   

    waitcnt(cnt + clkfreq/10)




    