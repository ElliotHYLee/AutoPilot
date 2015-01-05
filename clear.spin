CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
OBj
  usb            : "Parallax Serial Terminal"

pub main | x
  usb.start(115200)
  
  repeat
    x:=20010
    usb.dec( x//10000)
    usb.newline

  