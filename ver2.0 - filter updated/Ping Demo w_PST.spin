'' ***************************************
'' * Ping))) Demo with PST & LED's       *
'' * Author: Parallax Staff              *
'' * Copyright (c) 2006 Parallax, Inc.   *
'' * See end of file for terms of use.   *    
'' * Started: 06-03-2010                 *
'' ***************************************

{{

Code Description : In this example there are two LED's to indicate a distance. If the distance is further
 than 6 inches than LED 1 will turn on, and if the distance is closer than 6 inches than LED 2 turns on; while
 either LED 1 or 2 is on, the alternate LED will be off. There is a numerical display of the values in the
 Parallax Serial Terminal (PST) at 9600 baud (true).

}}

CON

  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

  PING_Pin      = 8                                          ' I/O Pin For PING)))
  LED1          = 1                                          ' I/O PIN for LED 1
  LED2          = 2                                          ' I/O PIN for LED 2

  ON            = 1
  OFF           = 0                                          
  Distlimit     = 6                                          ' In inches
  
VAR

  long  range

OBJ

  Debug  : "FullDuplexSerial"
  ping   : "ping"
  
PUB Start

  dira[LED1..LED2]~~
  outa[LED1..LED2]~

  Debug.start(31,30,0,9600)
  waitcnt(clkfreq + cnt)
    
  repeat                                                ' Repeat Forever

    debug.str(string(1,"PING))) Demo ", 13, 13, "Inches = ", 13, "Centimeters = ", 13))

    debug.str(string(2,9,2))
    range := ping.Inches(PING_Pin)                      ' Get Range In Inches
    debug.dec(range)
    debug.tx(11)

    debug.str(string(2,14,3))
    range := ping.Millimeters(PING_Pin)                 ' Get Range In Millimeters
    debug.dec(range / 10)                               ' Print Whole Part
    debug.tx(".")                                       ' Print Decimal Point
    debug.dec(range // 10)                              ' Print Fractional Part
    debug.tx(11)

    range := ping.Inches(PING_Pin)                      ' Get Range In Inches
  
   if range < Distlimit                                 ' Comparing range to a set value of 6 inches
     outa[LED1] := ON                                      ' P1 is on              
     outa[LED2] := OFF                                     ' P2 is off
   elseif range > Distlimit                             ' If range is further than 6 inches
     outa[LED1] := OFF                                     ' P1 is off    
     outa[LED2] := ON                                      ' P2 is on


{{
┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                                   TERMS OF USE: MIT License                                                  │                                                            
├──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation    │ 
│files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,    │
│modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software│
│is furnished to do so, subject to the following conditions:                                                                   │
│                                                                                                                              │
│The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.│
│                                                                                                                              │
│THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          │
│WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         │
│COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   │
│ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
}}        