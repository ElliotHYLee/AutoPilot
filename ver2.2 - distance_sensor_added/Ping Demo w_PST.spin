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

  PING_Pin      = 8 
  
VAR

  long  range

OBJ

  Debug  : "FullDuplexSerial"
  ping   : "ping"
  
PUB Start

  Debug.start(31,30,0,115200)

  waitcnt(clkfreq + cnt)
    
  repeat                                                ' Repeat Forever
       {
    'debug.str(string(1,"PING))) Demo ", 13, 13, "Inches = ", 13, "Milimeters = ", 13))

    'debug.str(string(2,9,2))
    'range := ping.Inches(PING_Pin)                      ' Get Range In Inches
    'debug.dec(range)
    'debug.tx(11)
     }
    debug.str(string(2,14,3))
    range := ping.Millimeters(PING_Pin)                 ' Get Range In Millimeters
    debug.dec(range)                               ' Print Whole Part


    'range := ping.Inches(PING_Pin)                      ' Get Range In Inches

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