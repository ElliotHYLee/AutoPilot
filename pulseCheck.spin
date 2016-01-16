CON
        _clkmode = xtal1 + pll16x                                               'Standard clock mode * crystal frequency = 80 MHz
        _xinfreq = 5_000_000

PIN = 8

VAR
  long  symbol
   
OBJ

  uart : "FullDuplexSerial.spin"

PUB main

  uart.quickStart   
  dira[pin]~

  repeat
     

  