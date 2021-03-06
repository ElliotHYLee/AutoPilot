CON

  _clkmode      = xtal1 + pll16x
  _xinfreq      = 6_250_000

  datapin  = 1   'SDA
  clockPin = 0   'SCL

 
 '' All available registers on the HMC5883 are listed below: (Check datasheet for detailed information)
  
  WRITE_DATA     = $3C 'Used to perform a Write operation
  READ_DATA      = $3D 'Used to perform a Read operation

  CNFG_A         = $00 'Read/Write Register, Sets Data Output Rate. Default = 15Hz & 8 samples per measurement 
                       '160Hz can be achieved by monitoring DRDY pin in single measurement mode.
  CNFG_B         = $01 'Read/Write Register, Sets the Device Gain(230-1370 Gauss). Default = 1090 Gauss
  MODE           = $02 'Read/Write Register, Selects the operating mode. Default = Single measurement
                       'Send $3C $02 $00 on power up to change to continuous measurement mode.         
  OUTPUT_X_MSB   = $03 'Read Register, Output of X MSB 8-bit value. (Will read -4096 if math overflow during bias measurement)
  OUTPUT_X_LSB   = $04 'Read Register, Output of X LSB 8-bit value. (Will read -4096 if math overflow during bias measurement)
  OUTPUT_Z_MSB   = $05 'Read Register, Output of Z MSB 8-bit value. (Will read -4096 if math overflow during bias measurement)
  OUTPUT_Z_LSB   = $06 'Read Register, Output of Z LSB 8-bit value. (Will read -4096 if math overflow during bias measurement)
  OUTPUT_Y_MSB   = $07 'Read Register, Output of Y MSB 8-bit value. (Will read -4096 if math overflow during bias measurement)
  OUTPUT_Y_LSB   = $08 'Read Register, Output of Y LSB 8-bit value. (Will read -4096 if math overflow during bias measurement)
  STATUS         = $09 'Read Register, indicates device status. 
  ID_A           = $0A 'Read Register, (ASCII value H)              
  ID_B           = $0B 'Read Register, (ASCII value 4)              
  ID_C           = $0C 'Read Register, (ASCII value 3)

VAR

long x
long y
long z, mag[3]

byte NE
byte SE
byte SW
byte NW


OBJ

    Term      :   "FullDuplexSerial"
    math      :   "SL32_INTEngine_2"
         
PUB Main | base

 ' waitcnt(clkfreq/100_000 + cnt)      'Wait while compass has time to startup.

  term.start(31, 30, 0, 115200)         'start a terminal Object (rxpin, txpin, mode, baud rate)

  setcont                             'sets 
  
  repeat
    
     term.tx(1) 
     base:= cnt
     updateMag                          'Gather raw data from compass
     term.dec(clkfreq/(cnt-base))
     term.tx(13)
                                       'Set Terminal data at top of screen


     RawTerm                          'Terminal window display X,Y,Z Raw Data
     HeadingTerm                      'Terminal window display of heading in degrees.
     waitcnt(cnt+clkfreq/10)


PUB updateMag

  SetCont
  setpointer(OUTPUT_X_MSB)         'Start with Register OUT_X_MSB
  getRaw
  mag[0] := x  - (5)
  mag[1] := y  - (-179)
  mag[2] := z  - (23)


PUB getMagX

  return mag[0]

PUB getMagY
  return mag[1]
PUB getMagZ
  return mag[2]

     
PUB HeadingTerm

  ''Terminal window display of heading in degrees.
   
     term.str(string("Heading in Degrees:",11))
     term.tx(13)
     term.tx(13)
      heading                                 
    

PUB AzimuthTerm

 ''Terminal window display of calculated arcTan(y/x)

     term.str(string("This is the calculated azimuth:",11))
     term.tx(13)
     term.tx(13)
     term.str(@Azm)
     term.dec(azimuth)               
     term.tx(13)
     term.tx(13)

PUB RawTerm

  '' Terminal window display X,Y,Z Raw Data
     term.dec(mag[0])        
     term.tx(13)
     term.dec(mag[1])
          term.tx(13)
     term.dec(mag[2])
          term.tx(13)

 
PUB aziadjust  : value

{{ Converts the Azimuth to Degrees from 0 - 360. }}

NW~
NE~
SE~
SW~
 
  if  x =< 0 
        if azimuth =< 0
           value := AZ_A
           NW := 1
        else
            NW~
            
        if azimuth > 0
           value := AZ_D
           SW := 1
        else
            SW~          
           
  if x > 0 
        if azimuth =< 0
          value := AZ_B
          NE := 1
        else
           NE~

                     
        if azimuth > 0
          value := AZ_C
          SE := 1
        else
            SE~
      
  value := 1 #> value <# 360

  value := (value + 270) // 360

  
  
PUB Heading   | t1, t2

 '' Gives a heading in alpha numeric format. From 0 - 90 degrees for NE,NW,SE,SW directions.
 t1~
 t2~    
 t1 := aziadjust
 
    if NE == 1

       term.str(@N)
       t2 := aziadjust
       term.dec(t2)  
       term.str(@E)
       term.tx(11) 
        
    if SE == 1

        term.str(@S)
        t2 := aziadjust - 180
        term.dec(||t2)  
        term.str(@E)
        term.tx(11) 
         
    if SW == 1

        term.str(@S)
        t2 := aziadjust - 180
        term.dec(t2)  
        term.str(@W)
        term.tx(11) 
         
    if (NW == 1)

        term.str(@N)
        t2 := 360 - aziadjust <#90
        if t2 == 90
         t2 := 0
        term.dec(t2)
        term.str(@W)
        term.tx(11) 

    
    if (t1 == 0)
        term.tx(11)
        term.tx(8)
        term.tx(8)
        term.tx(8)
        term.str(@NORTH)
        term.tx(11)    
      
       
    if t1 == 90
         term.tx(12)
         term.tx(8)
         term.tx(8)
         term.tx(8)
         term.tx(8)
         term.str(@EAST)
         term.tx(11) 
    

    if t1 == 180     
         
         term.tx(12)
         term.tx(8)
         term.tx(8)
         term.tx(8)
         term.str(@SOUTH)
         term.tx(11)
     

    if t1 == 271     
         term.tx(12)
         term.tx(8)
         term.tx(8)
         term.tx(8)
         term.tx(8)
         term.str(@WEST)
         term.tx(11) 

          
PUB SetCont

{{ Sets the Compass to Continuous output mode.}}
 
  start
  send(WRITE_DATA)
  send(MODE)
  send($00)
  stop

PUB SetPointer(Register)

{{ Start pointer at user specified Register. }}

start
send(WRITE_DATA)
send(Register)
stop

PUB GetRaw 

{{ Get raw data from continous output.}}

  start
  send(READ_DATA)
  x := ((receive(true) << 8) | receive(true))            'RegisterA and RegisterB
  z := ((receive(true) << 8) | receive(true))
  y := ((receive(true) << 8) | receive(false))
  stop
  ~~x
  ~~z
  ~~y
  x := x       
  z := z 
  y := y   
    
PRI Azimuth

    'Azimuth = arcTan(y/x)

           result := math.arctan(y,x)

PRI AZ_A        'NE
    
               result := ||azimuth 
                  
PRI AZ_B        'SE
   
               result := azimuth  + 180
     
PRI AZ_C        'SW
     
               result := azimuth  + 180
        
PRI AZ_D        'NW
     
               result := -azimuth + 360 
   
PRI send(value) ' I²C Send data - 4 Stack Longs

  value := ((!value) >< 8)

  repeat 8
    dira[dataPin]  := value
    dira[clockPin] := false
    dira[clockPin] := true
    value >>= 1

  dira[dataPin]  := false
  dira[clockPin] := false
  result         := !(ina[dataPin])
  dira[clockPin] := true
  dira[dataPin]  := true

PRI receive(aknowledge) ' I²C receive data - 4 Stack Longs

  dira[dataPin] := false

  repeat 8
    result <<= 1
    dira[clockPin] := false
    result         |= ina[dataPin]
    dira[clockPin] := true

  dira[dataPin]  := aknowledge
  dira[clockPin] := false
  dira[clockPin] := true
  dira[dataPin]  := true

PRI start ' 3 Stack Longs

  outa[dataPin]  := false
  outa[clockPin] := false
  dira[dataPin]  := true
  dira[clockPin] := true

PRI stop ' 3 Stack Longs

  dira[clockPin] := false
  dira[dataPin]  := false

DAT
        
E       byte  "E",0
N       byte  "N",0
S       byte  "S",0
W       byte  "W",0


Azm     byte "Azimuth = ",0
XRaw       byte "X = ",0
YRaw       byte "Y = ",0
ZRaw       byte "Z = ",0

NORTH   byte  "NORTH",0
SOUTH   byte  "SOUTH",0
EAST   byte  "EAST",0
WEST   byte  "WEST",0 