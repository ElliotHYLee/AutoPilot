{Object_Title_and_Purpose}


CON
  _clkmode = xtal1 + pll16x                                               'Standard clock mode * crystal frequency = 80 MHz
  _xinfreq = 5_000_000
  SIGNAL = 8
VAR
  long  stack[128], ping, gctra, dummy
   
OBJ
  uart : "FullDuplexSerial.spin"
  
PUB public_method_name    

  uart.quickStart
                        'A pin
  cognew(report, @stack) 

  ctra := %01000_000  << 23 + SIGNAL 'Establish mode and APIN (BPIN is ignored)
  frqa := 1
  dira[8]~

  gctra := ctra
  
  repeat
    if (dummy := phsa) ' if pin is high and, obviously, phsa is not 0
      if dummy == phsa  ' if pin is low(phsa stopped increasing)
        ping := phsa~    ' update ping and clear phsa


PUB report

  repeat
    uart.clear
    uart.decln(ping)
    waitcnt(cnt + clkfreq/10)


  {  
PRI private_method_name


DAT
name    byte  "string_data",0        
   }     