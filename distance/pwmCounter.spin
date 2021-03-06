{Object_Title_and_Purpose}


CON
        _clkmode = xtal1 + pll16x                                               'Standard clock mode * crystal frequency = 80 MHz
        _xinfreq = 5_000_000
  pw_in = 0
VAR
  long  symbol
   
OBJ
 ' nickname      : "object_name"
  uart : "FullDuplexSerial.spin"
PUB public_method_name | dummy, ping 

  uart.quickStart
                        'A pin
dira[pw_in]~    
ctra := %01000  << 23 + pw_in 'Establish mode and APIN (BPIN is ignored)


repeat
  uart.clear
  dummy := phsa
  uart.decln(dummy)
  if (dummy) ' if pin is high and, obviously phsa is not 0
    if dummy == phsa  ' if pin is low
      ping := phsa~    ' update ping and clear phsa
      uart.decln(ping)
  waitcnt(cnt + clkfreq/10)
      
PRI private_method_name


DAT
name    byte  "string_data",0        
        