CON
_clkmode = xtal1 + pll16x
_xinfreq = 5_000_000

  FF                            = 12                    ' form feed
  CR                            = 13                    ' carriage return
  NL                            = 13

  usb = 0 
  usbBaud = 115200  
  usbRx = 31
  usbTx = 30

  xb = 1
  xbBaud = 9600
  xbRx = 1
  xbTx = 0

  
OBJ
  com :  "FullDuplexSerial4port"

PUB main | isReceived, char

  initialize

  repeat
    isReceived := com.rxIsIn(usb)
    if isReceived
      char :=getc(usb)
      putc(usb, char)
      newline(usb)


PUB initialize

  com.AddPort(usb, usbRx, usbTx,-1,-1,0,0,usbBaud)
  com.AddPort(xb, xbRx, xbTx, -1, -1, 0, 0, xbBaud)
  com.start
  pause(100)


PUB pause(ms)
  waitcnt(clkfreq/1000*ms + cnt)


PUB str(port,stringptr)
'' Send string                    
  repeat strsize(stringptr)
    com.tx(port,byte[stringptr++])

PUB newline(port)
  putc(port,CR)

PUB putc(port,txbyte)
'' Send a byte to the terminal
  com.tx(port,txbyte)

PUB getc(port)
'' Get a character
'' -- will not block if nothing in uart buffer
   return com.rxcheck(port)
'  return rx

PUB dec(port,value) | i
'' Print a decimal number
  decl(port,value,10,0)

PUB decl(port,value,digits,flag) | i, x
  digits := 1 #> digits <# 10
  x := value == NEGX                                                            'Check for max negative
  if value < 0
    value := ||(value+x)                                                        'If negative, make positive; adjust for max negative
    com.tx(port,"-")

  i := 1_000_000_000
  if flag & 3
    if digits < 10                                      ' less than 10 digits?
      repeat (10 - digits)                              '   yes, adjust divisor
        i /= 10
  repeat digits
    if value => i
      com.tx(port,value / i + "0")
      value //= i
      result~~
    elseif (i == 1) OR result OR (flag & 2)
      com.tx(port,"0")
    elseif flag & 1
      com.tx(port," ")
    i /= 10
  