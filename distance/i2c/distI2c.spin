CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

OBJ

 usb : "FullDuplexSerial.spin"
 i2c : "I2C Spin push_pull driver" 

pub main | x

  usb.quickStart

  repeat
    usb.dec(10)

  
Pub



