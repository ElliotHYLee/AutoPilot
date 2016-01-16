{Object_Title_and_Purpose}


CON
  _clkmode = xtal1 + pll16x                                               'Standard clock mode * crystal frequency = 80 MHz
  _xinfreq = 5_000_000
  SIGNAL = 8
VAR
  long  stack[128], ping
   
OBJ
  uart : "FullDuplexSerial.spin"
  
PUB public_method_name | dummy  

  uart.quickStart
                        'A pin
  cognew(report, @stack) 

  'ctra := %01000_000 << 23 + 1 << 9 + 8 'Establish mode and APIN (BPIN is ignored) 
  ctra := %01000_000  << 23 + SIGNAL 'Establish mode and APIN (BPIN is ignored)

  dira[8]~
  

  repeat
    dummy := phsa
    if (dummy) ' if pin is high and, obviously phsa is not 0
      if dummy == phsa  ' if pin is low
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