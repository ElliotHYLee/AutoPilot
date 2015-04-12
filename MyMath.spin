CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

ObJ

  usb : "FullDuplexSerial"

PUB main 

  usb.quickStart

  repeat
    
    usb.decLn(sqrt(1234))


PUB sqrt(value)| x, i

  x := value

  repeat i from 0 to 20
    x := (value/x + x) /2

  return x


PUB getSign(value)

  if value >= 0
    result := 1
  else
    result := -1

PUB getAbs(value)
  if value > 0
    result := value
  else
    result := -value
    