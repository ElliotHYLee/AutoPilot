{{
┌───────────────────────────────────────────────┐
│ Laser Range Finder: Basic Demonstration       │
│                                               │
│ Author: Joe Grand [www.grandideastudio.com]   │
│ Contact: support@parallax.com                 │ 
│                                               │
│ See end of file for terms of use.             │                      
└───────────────────────────────────────────────┘

Program Description:

This program demonstrates the Parallax Laser Range Finder (LRF) module.
The distance to the target object is displayed in the Parallax Serial Terminal.  


Laser Range Finder Connector (4 pin male header)
────────────────────────────────────────────────

SIN     ────── P6
SOUT    ────── P7
VCC     ────── +5V (VDD)
GND     ──────┐
               
              GND (VSS)


Revisions:

1.1 (July 2011): Initial release
 
}}


CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

  LRF_TX        = 6             ' Serial output to LRF (connects to SIN)
  LRF_RX        = 7             ' Serial input from LRF (connects to SOUT)
 
VAR
  long range                    ' Distance from LRF module to target object (in millimeters)
    
OBJ
  pst       : "Parallax Serial Terminal"                   ' Debug Terminal
  serial    : "Extended_FDSerial"                          ' Extended Full Duplex Serial by Martin Hebel (http://obex.parallax.com/objects/31/)
  
PUB main
  pst.Start(115_200)            ' Set Parallax Serial Terminal to 115.2kbps             
  pst.Str(@InitHeader)          ' Print header; uses string in DAT section.

  ' Set-up serial port for communication with the LRF module
  serial.Start(LRF_RX, LRF_TX, %0000, 115_200)          ' Start serial port, normal mode, 115.2kbps
  
 {{
   When the LRF powers on, it launches an auto-baud routine to determine the
   host's baud rate. It will stay in this state until a "U" ($55) character
   is sent by the host.
 }}
  pst.Str(String("Waiting for LRF module..."))
  waitcnt(clkfreq << 1 + cnt)                           ' Delay for 2 seconds to let the LRF module start-up
  serial.Tx("U")                                        ' Send character
  repeat until serial.RxCheck == ":"                    ' When the LRF has initialized and is ready to go, it will send a single ':' character, so wait here until we receive it
  pst.Str(String("Ready!", pst#NL, pst#NL))             ' Ready to go!

  repeat
    {{
      When a single range (R) command is sent, the LRF returns the distance to the target
      object in ASCII in millimeters. For example:

      D = 0123
    }}

    serial.Tx("R")                                      ' Send command
    repeat until serial.RxCheck == "D"                  ' Wait for the header to be sent...
    repeat until serial.RxCheck == "="                  '
    repeat until serial.RxCheck == " "                  '              
    range := serial.RxDec                               ' ...then grab the value
    
    ' With the data in hand, let's display it...
    pst.Str(String("Distance = "))
    pst.Dec(range / 10)
    pst.Char(".")
    pst.Dec(range // 10)
    pst.Str(String(" cm, "))                            ' ... in centimeters

    range := (range * 10) ** 169_093_200                ' convert mm to hundredths of an inch (169_093_200 = 1 / 25.4 * $FFFFFFFF)
    pst.Dec(range / 10)
    pst.Char(".")
    pst.Dec(range // 10)
    pst.Str(String(34, pst#CE, pst#NL, pst#MU))         ' ...and in inches

DAT
InitHeader    byte "Parallax Laser Range Finder", pst#NL 
              byte "Basic Range Demonstration", pst#NL, pst#NL, 0


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