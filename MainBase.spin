CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

OBJ
  myAutoPilot : "AutoPilot.spin"

PUB main
  myAutoPilot.getReady
  myAutoPilot.startAutoPilot
    