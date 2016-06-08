{{
SD_Card_Micro_01.spin

Copyright (c) 2011 Greg Denson
See end of file for terms of use 

Based on a tutorial by Jeff Ledger (Original tutorial can be found at:  http://gadgetgangster.com/tutorials/331)

**** THIS PROGRAM WORKS WELL FOR BASIC MICRO SD CARD COMMUNICATION - I USE IT AS A TEMPLATE FOR MICRO SD CARDS OBJECTS ****

Original Program Created By:
Jeff Ledger (Gadget Gangster Tutorial)

Modified Program Created By:
Greg Denson, 2011-05-30 to run with my PPD Board and SD card holder.

Modified By:
Greg Denson, 2011-06-29 to use the PPD Board with the Parallax Micro SD Card adapter/holder.

}}

CON
  _clkmode = xtal1 + pll16x             ' Set up the clock frequencies
  _xinfreq = 6_250_000

  CS  = 8       ' Propeller Pin 3 - Set up these pins to match the Parallax Micro SD Card adapter connections.
  DI  = 11       ' Propeller Pin 2 - For additional information, download and refer to the Parallax PDF file for the Micro SD Adapter.                        
  CLK = 13       ' Propeller Pin 1 - The pins shown here are the correct pin numbers for my Micro SD Card adapter from Parallax                               
  D0  = 12       ' Propeller Pin 0 - In addition to these pins, make the power connections as shown in the following comment block.
Var

  long mag[3]
  long timerStack[100], debugStack[100]
  long clock, clockLim, base, dt, flag
                  
OBJ
  sd      : "fsrw"                               ' Download the fswr.spin object from the Parallax Propeller Object Exchange (OBEX), accessed from parallax.com
  debug   : "fullDuplexSerial4Port_tier2.spin"   ' If you don't already have it in your working directory, you can also download this object from OBEX.
  sensor  : "NewMag.spin"
PUB main | insert_card, text

  clockLim := 60
  waitcnt(cnt + clkfreq*2)

  debug.quickStartDebug
  
  
  insert_card := \sd.mount_explicit(D0, CLK, DI, CS)        ' Here we call the 'mount' method using the 4 pins described in the 'CON' section.
  if insert_card < 0                                           ' If mount returns a zero...
    debug.str(0,string(13))                                        ' Print a carriage return to get a new line.
    debug.str(0,string("The Micro SD Card was not found!"))        ' Print the failure message.
    debug.str(0,string(13))                                        ' Carriage return...
    debug.str(0,string("Insert card, or check your connections.")) ' Remind user to insert card or check the wiring.
    debug.str(0,string(13))                                        ' And yet another carriage return.
    abort                                                      ' Then we abort the program.
  debug.str(0,string(13))
  debug.str(0,string("Micro SD card was found!"))                  ' Let the user know the card is properly inserted.
  debug.str(0,string(13))
  
  sd.popen(string("output.txt"), "w")  
  debug.str(0, String("File Opened"))

  cognew(timer, @timerStack)
  cognew(runDebug, @debugStack)
    
  repeat while flag > 0
    base:=cnt
    update
    sd.dec(mag[0])
    sd.str(String(" ")) 
    sd.dec(mag[1])
    sd.str(String(" "))
    sd.dec(mag[2])
    sd.str(String(" "))
    sd.newline
    waitcnt(cnt + (clkfreq/60-(cnt-base)))
    dt := cnt - base  
  sd.pclose                            
  sd.unmount                           ' This line dismounts the card so you can safely remove it.


  repeat 100
    debug.str(0, String("end"))
    waitcnt(cnt+ clkfreq/10)



PUB timer
  flag := 1
  repeat clockLim
    waitcnt(cnt + clkfreq)
    clock++
  flag := 0
    
PRI update
  sensor.updateMag
  mag[0] := sensor.getMagX
  mag[1] := sensor.getMagY
  mag[2] := sensor.getMagZ 

PUB runDebug
  repeat while flag > 0
    debug.clear(0)
    debug.str(0,string("dt = "))
    debug.decln(0,dt)
    debug.str(0,string("freq = "))
    debug.decln(0,clkfreq/dt)
    debug.str(0,String("Current Time: "))
    debug.dec(0,clock)
    debug.str(0,String("/"))
    debug.dec(0,clockLim)
    debug.strLn(0,String(" sec"))

    debug.str(0,String("mag[0]: "))
    debug.decLn(0,mag[0])
    debug.str(0,String("mag[1]: "))
    debug.decLn(0,mag[1])
    debug.str(0,String("mag[2]: "))
    debug.decLn(0,mag[2])
    debug.newline(0)      
    waitcnt(cnt + clkfreq/10)




    