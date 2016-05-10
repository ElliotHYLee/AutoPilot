CON
_clkmode = xtal1 + pll16x
_xinfreq = 5_000_000

VAR

  long cog_kinect, stack_kinect[128], cog_xbee, stack_xbee[128]
  long cog_prop2prop, stack_prop2prop[128]

  long mag[3], acc[3], gyro[3], eAngle[3], refAtt[3], newValueCounter
  long pulse[6], throttle, distGroundFiltered

  long xOn, xOff, yOn, yOff, zOn, zOff
  long xKp, xKi, xKd, xOutput, xPro, xDer, xInt
  long yKp, yKi, yKd, yOutput, yPro, yDer, yInt
  long zKp, zKi, zKd, zOutput, zPro, zDer, zInt

  long lc[3] 'local coordinate pointer

  long systemMode, respondType, respondContent              
  long varchar, varchar2, newValue, type,motorNumber,pidUpdateIndex
  long lcAxisNumber, coord
  long pidAxis
  long pidOnOff[3], navPidOnOff[3] 
  
OBJ

  kinect : "CommKinect.spin"
  xbee   : "CommXbee.spin"
  prop   : "CommProp.spin"

PUB secondaryPropMain
'===============================================
' This code is for secondary propeller   
'===============================================


  startKinect          ' 2 cogs for Kinect receiving

  startXbee            ' 2 cogs for Xbee communication

  startProp2Prop       ' 2 cogs for prop to prop communication
                 


PRI startProp2Prop

  prop.initialize 
  prop.setAttPtr(@acc, @gyro, @eAngle, @mag)
  prop.setMotPtr(@pulse)
  prop.setThrottle(@throttle)
  'prop.setXPidPtr(@xKp, @xKd, @xKi, @xPro, @xDer, @xInt, @xOutput)
  'prop.setYPidPtr(@yKp, @yKd, @yKi, @yPro, @yDer, @yInt, @yOutput)
  'prop.setZPidPtr(@zKp, @zKd, @zKi, @zPro, @zDer, @zInt, @zOutput)
  prop.setPidOnOffPtr(@pidOnOff)
  prop.setTargetAttitude(@refAtt)
  prop.setDistPtr(@distGroundFiltered)
  prop.setNavPidOnOffPtr(@navPidOnOff)

  cog_prop2prop := cognew(runProp2Prop, @stack_prop2prop)
  
PRI runProp2Prop

  prop.communicate



 
'===============================================
' Xbee Communication
'===============================================
PRI startXbee

  xbee.initialize
  xbee.setAttPtr(@acc, @gyro, @eAngle, @mag)
  xbee.setMotPtr(@pulse)
  xbee.setThrottle(@throttle)
  xbee.setXPidPtr(@xKp, @xKd, @xKi, @xPro, @xDer, @xInt, @xOutput)
  xbee.setYPidPtr(@yKp, @yKd, @yKi, @yPro, @yDer, @yInt, @yOutput)
  xbee.setZPidPtr(@zKp, @zKd, @zKi, @zPro, @zDer, @zInt, @zOutput)
  xbee.setPidOnOffPtr(@pidOnOff)
  xbee.setTargetAttitude(@refAtt)
  xbee.setDistPtr(@distGroundFiltered)
  xbee.setNavPidOnOffPtr(@navPidOnOff)

  cog_xbee := cognew(runXbeeComm, @stack_xbee)  

PRI runXbeeComm

 xbee.communicate

'===============================================
' Kinect Communication
'===============================================
PRI startKinect

  kinect.initialize
  kinect.setLocalCoordinate(@lc)
  cog_kinect := cognew(runKinectComm, @stack_kinect)

PRI runKinectComm

  kinect.communicate



  