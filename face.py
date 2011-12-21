#!/opt/grx/bin/hrpsyspy

# the z offset  of desk from the robot base
# !!! PLEASE MEASURE THIS VALUE FIRST !!!
DEFAULT_BASE_OFFSET_Z = -0.095
# !!! PLEASE MEASURE THIS VALUE FIRST !!!

import sys
import time
import rtm
import sample
import types
import math
import Img.CameraCaptureServiceHelper
import OpenHRP.CvProcessorServiceHelper
import java.lang.System
import OpenHRP.RobotHardwareServicePackage.RobotState

import bodyinfo

global conf

POSITION_X_LIMIT = 0.375
FRAME_IDX=0

#############################
#
# Rates
#
rate70 = 70
rate60 = 60
rate50 = 50
rate40 = 40
rate30 = 30
rate20 = 20
rate10 = 10
rate5  = 5
rateDefault=rate40

#############################

#############################
#
# Parameters
#
class DemoConfig:
  def __init__(self, baseOffsetZ = DEFAULT_BASE_OFFSET_Z):
    self.baseOffsetZ          = baseOffsetZ
    self.handLengthZ          = 0.103
    #self.trayHandleZ          = 0.080
    #self.gripDepthZ           = 0.050
    self.trayHandleZ          = 0.110
    self.gripDepthZ           = 0.044
    self.ballShuffleT         = 3
    self.trayHandleClearanceX = 0.045
    self.searchMovementX      = 0.010	
    self.ballGripAdjX         = 0.003
    self.dropMarginY          = 0.0
    # the x,z displacement to shuffle tray in absolute value [m]
    self.shuffleTrayDX        = 0.015
    # the absolute position x to push/pull tray [m]
    self.pullTrayX            = 0.135 #<- based on initial position of moveTray
    self.pushTrayX            = 0.20
    self.ballClearance        = 0.05

    # number of pixels
    self.heightView           = 480
    self.widthView            = 640
    
    # the distance between camera center and hand center
    self.cameraOffsetX        = 0.035 #for HIRO
    
    # the z displacement to pick ball in absolute value [m]
    self.pickBallZ            =-0.015 # 0.030, 0.000
    
    # contol values for 5% and 3% diffence [m]
    self.controlValueX05      = 0.01
    self.controlValueX03      = 0.002
    self.controlValueY05      = 0.01
    self.controlValueY03      = 0.002
    
    # contol value z to search the target
    self.controlValueZ        = 0.02

    self.update(baseOffsetZ)

  def update(self, baseOffsetZ):
    self.lowerLimitX          = 0.3
    self.upperLimitX          = 0.5
    self.lowerLimitYL         =-0.05
    self.upperLimitYL         = 0.3
    self.lowerLimitYR         =-self.lowerLimitYL
    self.upperLimitYR         =-self.upperLimitYL
    self.lowerLimitZ          = self.baseOffsetZ + self.handLengthZ + self.ballClearance
    self.upperLimitZ          = self.lowerLimitZ - self.ballClearance + 0.170

    # the z displacement to pick ball in absolute value [m]
    #self.pickTrayZ            = 0.074
    self.pickTrayZ            = self.baseOffsetZ + self.trayHandleZ - self.gripDepthZ + self.handLengthZ

SLEEP_ONE_BLOCK		= 60
MAX_RESET_COUNT		= 2
MAX_RETRY_COUNT		= 15
#############################

#############################
#
# Local Log File
#
ik_errorlog_filename = '/tmp/ikerror'

def speak(s):
  import os
  if type(s) == types.StringType:
    os.system('python speak.py "'+s+'"')
    #os.system('spd-say "'+s+'"')
  else:
    print 'error(speak): input is not a string'

def logIkErrorPos(x,y,z,r,p,w):
  global ik_errorlog_filename
  of = open(ik_errorlog_filename, 'a')
  of.write('%f,%f,%f,%f,%f,%f\n'%(x,y,z,r,p,w))
  of.close()

def init(host='localhost'):
  global vs, vs_svc, cvp, cvp_svc
  global vs_head, vs_head_svc, cvp_head, cvp_head_svc

  sample.init(host)

  # for hand
  vs = rtm.findRTC("VideoStream0")
  if vs != None:
    vs_svc = Img.CameraCaptureServiceHelper.narrow(vs.service('service0'))
    vs.start()

    cvp = rtm.findRTC("CvProcessor0")
    cvp_svc = OpenHRP.CvProcessorServiceHelper.narrow(cvp.service('service0'))
    cvp.start()

    rtm.connectPorts(vs.port("MultiCameraImages"),   cvp.port("MultiCameraImage"))
    vs_svc.take_one_frame()

  # for head
  vs_head = rtm.findRTC("VideoStream0_head")
  if vs_head != None:
    vs_head_svc = Img.CameraCaptureServiceHelper.narrow(vs_head.service('service0'))
    vs_head.start()

    cvp_head = rtm.findRTC("CvProcessor0_head")
    cvp_head_svc = OpenHRP.CvProcessorServiceHelper.narrow(cvp_head.service('service0'))
    cvp_head.start()

    rtm.connectPorts(vs_head.port("MultiCameraImages"),   cvp_head.port("MultiCameraImage"))
    vs_head_svc.take_one_frame()

  time.sleep(1)

def getControlValue(comX, comY):
  # determine the control values
  dx = 0.
  dy = 0.
  if abs(comX)   > 0.05:
    dx = conf.controlValueX05
  elif abs(comX) > 0.03: # used to be 0.02 
    dx = conf.controlValueX03
  if comX > 0:
    dx *= -1

  if abs(comY)   > 0.05:
    dy = conf.controlValueY05
  elif abs(comY) > 0.03: # used to be 0.02 
    dy = conf.controlValueY03
  if comY < 0:
    dy *= -1

  return dx,dy

def getCircles(color = 'orange'):
  NUM_TO_TAKE=1
  cvp.ref.get_configuration().activate_configuration_set(color)
  x = 0
  y = 0
  r = 0
  success_count = 0
  ret = OpenHRP.darray3SeqHolder()
  for i in range(NUM_TO_TAKE):
    vs_svc.take_one_frame()
    time.sleep(0.1)
    cvp_svc.HoughCircles(FRAME_IDX, ret)
    if len(ret.value) > 0:
      success_count += 1
      maxval = 0 
      maxidx = 0
      for i in range(len(ret.value)):
        if maxval > ret.value[i][1]:
          maxval = ret.value[i][1]
          maxidx = i
        #print "%d = %6.3f,%6.3f unit:[m]"%(i, ret.value[i][1], ret.value[i][0])
      #print "target = %d"%(maxidx)
      x += ret.value[maxidx][1]
      y += ret.value[maxidx][0]
      r += ret.value[maxidx][2]
  if success_count > 0:
    x /= success_count*conf.heightView
    y /= success_count*conf.widthView
    r /= success_count*conf.widthView

  return x, y, r


def doLimit(curV, diffV, lwrLimitV, uprLimitV, valString, spkString):

  inLimit = False
  
  if (curV+diffV) < lwrLimitV:
    print valString+" lower limit"
    diffV = lwrLimitV - curV
    
  elif uprLimitV < (curV+diffV):
    print valString+" upper limit"
    diffV = uprLimitV - curV

  else:
    inLimit = True

  if not inLimit and spkString:
    speak('limited %s movement'%spkString)

  return diffV, inLimit


def pickBall(dropDy):
  x0,y0,z0,roll0,pitch0,yaw0 = \
      sample.getCurrentConfiguration(sample.armL_svc)
  dzz = conf.lowerLimitZ - z0
  if sample.moveRelativeL(dx= conf.cameraOffsetX, dz=dzz, rate=rate10) < 0:
    print "ik error."
    speak('eye kay error.')
    speak('pull the tray.')
    moveTray('pull')
    return False
  for i in range(5):
    sample.lhandOpen(50-i*5)
    time.sleep(0.25)
  sample.moveRelativeL(dz=-dzz, rate=rate60) # rate = 10
  sample.moveRelativeL(dy= dropDy, rate=rate70) # rate = 10 
  sample.lhandOpen(60)
  time.sleep(0.3)
  sample.moveRelativeL(dx=-conf.cameraOffsetX, dy=-dropDy, rate=rate70)
  return True

def moveTray(mode = 'shuffle'):
  global POSITION_X_LIMIT
  # rotate
  sample.moveRelativeL(dy=conf.cameraOffsetX, dw=-math.pi/2, rate=rate40)
  
  # move to conf.upperLimitZ
  x0,y0,z0,roll0,pitch0,yaw0 = sample.getCurrentConfiguration(sample.armL_svc)
  print "\n( x, y, z, r, p, w) = %6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f  unit:[m]"%(x0, y0, z0, roll0, pitch0, yaw0)
  sample.moveL(x0, y0, conf.upperLimitZ, roll0, pitch0, yaw0, rate=rateDefault)

  # detect handle
  lines   = OpenHRP.iarray4SeqHolder()
  okCount = 0
  searchDirec = -1
  searchDirec_x = -1
  while 1:
    if cvp != None:
      cvp.ref.get_configuration().activate_configuration_set('green')
      vs_svc.take_one_frame()
      time.sleep(0.1)
      cvp_svc.HoughLinesP(FRAME_IDX, lines)
      linesValue = lines.value
    else:
      linesValue = [[conf.widthView/2.0, conf.heightView/2.0, conf.widthView/2.0, conf.heightView/2.0]]
    dx = dy = 0.0
    if len(linesValue) > 0:
      ditectDelec = 0
      comX = comY = 0.0
      for pnt in linesValue:
        comX += pnt[0] + pnt[2]
        comY += pnt[1] + pnt[3]
      comX = comX / 2.0 / len(linesValue) / conf.widthView  - 0.5 + 0.01
      comY = comY / 2.0 / len(linesValue) / conf.heightView - 0.5 
      dx,dy = getControlValue(comX, comY)
      if dx == 0 and dy == 0:
        okCount += 1
        if okCount > 3:
          speak('handle detected.')
          break

      print "x[0]=%6.3f comX=%6.3f dx=%6.3f y[0]=%6.3f comY=%6.3f dy=%6.3f"%(linesValue[0][0], comX, dx, linesValue[0][0], comY, dy)
    else:
      #speak('detecting handle.')
      dy = searchDirec * 0.03
      dx = searchDirec_x * 0.03
      okCount = 0

    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    dx, inLimitX = doLimit(x1, dx, conf.lowerLimitX,  conf.upperLimitX,  'x', 'ecks')
    dy, inLimitY = doLimit(y1, dy, conf.lowerLimitYL, conf.upperLimitYL, 'y', 'wai')

    # change Y search direction -> do the same with X
    if len(linesValue) == 0:
      if not inLimitX:
        searchDirec_x *= -1
      if not inLimitY:
        searchDirec   *= -1

    if sample.moveL(x1+dx, y1+dy, z1, -math.pi/4,-math.pi/2,-math.pi/4,rate=rate40) < 0:
      print "ik error."
      speak('eye kay error.')
      logIkErrorPos(x1+dx, y1+dy, z1, -math.pi/4,-math.pi/2,-math.pi/4)

  #  open & down
  sample.lhandOpen(60)
  time.sleep(0.3)
  dzz = conf.pickTrayZ - z1 # TODO 
  sample.moveRelativeL(dy=-conf.cameraOffsetX, dz=dzz, rate=rate10)

  # grasp & do the action
  sample.lhandClose()
  time.sleep(0.3)
  if mode == 'pull':
    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    sample.moveL(0.35,y1,z1,roll1,pitch1,yaw1,rate=rate10)
  elif mode == 'shuffle':
    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    sample.moveL(0.35,y1,z1,roll1,pitch1,yaw1,rate=rateDefault)

    sample.moveRelativeL(dx= conf.shuffleTrayDX, dz= 0.06, rate=rate5)
    time.sleep(conf.ballShuffleT)
    sample.moveRelativeL(dx=-conf.shuffleTrayDX, dz=-0.06, rate=rate5)
  elif mode == 'push':
    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    sample.moveL(0.42,y1,z1,roll1,pitch1,yaw1, rate=rate10)

  # open & up & rotate
  sample.lhandOpen()
  time.sleep(0.3)
  x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
  sample.moveL(x1, y1, conf.upperLimitZ, roll1, pitch1, yaw1, rate=rate20)
  sample.moveRelativeL(dx=conf.cameraOffsetX, dw=math.pi/2, rate=rate40)

def loop(numTry=1):
  global cvp
  count = 0
  retryCount = 0
  shuffleCount = 0
  sample.lhandOpen(60)
  lastY = 0
  tryCount = 0
  numOrange = 0
  numBlue = 0

  x0,y0,z0,roll0,pitch0,yaw0 = sample.getCurrentConfiguration(sample.armL_svc)
  while 1:
    x,y,z,roll,pitch,yaw = sample.getCurrentConfiguration(sample.armL_svc)
    print "\n( x, y, z, r, p, w) = %6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f  unit:[m]"%(x, y, z, roll, pitch, yaw)

    if cvp != None:
      cx_orange, cy_orange, r_orange = getCircles('orange')
      cx_blue, cy_blue, r_blue = getCircles('blue')

      if cx_orange > cx_blue and r_orange > 0:
        cx = cx_orange  
        cy = cy_orange  
        r  = r_orange  
        detectColor = 'orange'
      elif r_blue > 0:
        cx = cx_blue
        cy = cy_blue
        r  = r_blue
        detectColor = 'blue'
      else:
        r = 0
    else:
      import random
      cx = random.random()*0.1+0.45
      cy = random.random()*0.1+0.45
      r  = random.random()*0.15
      if random.random() > 0.5:
        detectColor = 'orange'
      else:
        detectColor = 'blue'
      
    if r > 0.05: # circle found
      print detectColor
      shuffleCount = 0
      cx = -cx + 0.5
      cy = -cy + 0.5
      dz = 0.0
      dx,dy = getControlValue(-cx, cy)
      if r < 0.15:
        dz = -0.01

      # speak adjusting parameters

      if dx == 0 and dy == 0 and (dz <= conf.lowerLimitZ or dz == 0):
        count += 1
      elif count > 1:
        count -= 1

      if count > 2:
        # grasping
        lastY = y
        if detectColor == 'orange':
          speak("Picking orange ball.")
          dropDy =  0.1
          numOrange += 1
        else:
          dropDy = -0.1
          speak("Picking blue ball.")
          numBlue += 1
        pickBall(dropDy)
        count = 0;
      else:
        #
        print "count="+str(count)
        print "(cx,cy, r) = %6.3f,%6.3f,%6.3f  unit:-"%(cx, cy, r)
        dx /= count+1.0
        dy /= count+1.0
    else: 
      #if r > 0.00001: # circle found
      # speak('ignored tiny circle')
      # circle NOT founded
      count = 0;
      dx = dy = dz = 0
      if z + 0.02 < conf.upperLimitZ:
        dz = 0.02

      shuffleCount += 1
      if shuffleCount > 3:
        tryCount += 1
        if (numTry > 0) and (tryCount >= numTry):
          print 'finished '
          speak('I tried %s times'%(numTry))
          speak('orange balls are %s '%(numOrange))
          speak('blue balls are %s '%(numBlue))
          return
        print 'shuffling balls'
        speak('shuffling balls')
        moveTray('shuffle')
        x2,y2,z2,yaw2,roll2,pitch2 = sample.getCurrentConfiguration(sample.armL_svc)
        x = x2
        y = y2
        shuffleCount = 0
        numOrange = numBlue = 0
      
    rate = rateDefault
    print "(dx,dy,dz) = %6.3f,%6.3f,%6.3f  unit:[m]"%(dx, dy, dz)
    if 0: # irex 2011 for hiro
      if (x+dx<POSITION_X_LIMIT and dx<0) or (conf.upperLimitX<x+dx and 0<dx):
        print "x limit"
        speak('return to initial position')
        rate = rate10
        dx = x0 - x
        dy = y0 - y
        dz = z0 - z
      else:
        if (y+dy<conf.lowerLimitYL and dy<0) or (conf.upperLimitYL<y+dy and 0<dy):
          print "y limit"
          speak('limited wai movement')
          dy = 0
        if z+dz<conf.lowerLimitZ and dz<0:
          print "z lower limit"
          #speak('limited zed movement')
          dz = conf.lowerLimitZ - z
        elif conf.upperLimitZ<z+dz and 0<dz:
          print "z upper limit"
          dz = conf.upperLimitZ - z

      if sample.moveL(x+dx, y+dy, z+dz,0, -math.pi/2, 0, rate=rateDefault) < 0:
        print "ik error."
        speak('eye kay error.')
        logIkErrorPos(x+dx, y+dy, z+dz,0, -math.pi/2, 0)

    # limit motion
    dx,inLimitX = doLimit(x, dx, conf.pullTrayX + conf.trayHandleClearanceX, conf.upperLimitX, 'x', 'ecks')
    dy,inLimitY = doLimit(y, dy, conf.lowerLimitYL, conf.upperLimitYL, 'y', 'wai')
    dz,inLimitZ = doLimit(z, dz, conf.lowerLimitZ,  conf.upperLimitZ,  'z', None)

    if not inLimitX:
      limitHitX = limitHitX + 1
      if limitHitX > MAX_RESET_COUNT:
        return # give up
      else:
        #sample.moveL(x0, y0, z0, 0,-math.pi/2, 0, rate=rateDefault)
        return
        # should do IK check here
    elif sample.moveL(x+dx, y+dy, z+dz, 0, -math.pi/2, 0, rate=rateDefault) < 0:
      print "ik error."
      speak('eye kay error.')
      logIkErrorPos(x+dx, y+dy, z+dz, 0, -math.pi/2, 0)

    if bodyinfo.modelName == 'PARM':
      time.sleep(1.0)
      retryCount = retryCount + 1
      if retryCount > MAX_RETRY_COUNT:
        speak('over heating')
        return

def getNearestFace(doSaveImage=False):
  NUM_TO_TAKE=1
  x = 0
  y = 0
  r = 0
  success_count = 0
  ret = OpenHRP.darray3SeqHolder()
  for i in range(NUM_TO_TAKE):
    vs_head_svc.take_one_frame()
    time.sleep(0.1)
    cvp_head_svc.detectFaces(FRAME_IDX, ret, doSaveImage)
    if len(ret.value) > 0:
      success_count += 1
      maxval = 0 
      maxidx = 0
      for i in range(len(ret.value)):
        if maxval < ret.value[i][2]:
          maxval = ret.value[i][2]
          maxidx = i
      x += ret.value[maxidx][1]
      y += ret.value[maxidx][0]
      r += ret.value[maxidx][2]
  if success_count > 0:
    x /= success_count*conf.heightView
    y /= success_count*conf.widthView
    r /= success_count*conf.widthView

  return x, y, r

def loopDetectFace(num):
  okCount = 0
  time.sleep(1)
  i = 0
  while i != num:
    i += 1
    stat = OpenHRP.RobotHardwareServicePackage.RobotStateHolder()
    sample.rh_svc.getStatus(stat)
    pan0  = stat.value.command[1]*180.0/math.pi
    tilt0 = stat.value.command[2]*180.0/math.pi
    cx,cy,r = getNearestFace()
    faceRadiusMin = 0.05
    if r > faceRadiusMin:
      cx = 0.5 - cx
      cy = 0.75 - cy 

      # determine the control values
      dx = dy = 0
      if abs(cx)   > 0.10:
        dx = 0.02
      elif abs(cx) > 0.5: # used to be 0.02 
        dx = 0.002
      if cx > 0:
        dx *= -1
  
      if abs(cy)   > 0.10:
        dy = 0.01
      elif abs(cy) > 0.05: # used to be 0.02
        dy = 0.002
      if cy < 0:
        dy *= -1

      dx *= 100
      dy *= 100
      pan  = min(pan0+dy,  15)
      pan  = max(pan,     -15)
      tilt = min(tilt0+dx, 30)
      tilt = max(tilt,     10)
      print "dx,dy = %f %f"%(dx, dy)
    else:
      pan  =  0 
      tilt = 10

    print "pan, tilt, time : %f %f" %(pan, tilt)
    time2move = max(0.3, max(abs(pan-pan0),abs(tilt-tilt0))/5.0)
    sample.setJointAnglesDeg([[0,pan, tilt],[],[],[],[]], time2move, wait=True)

    if r > faceRadiusMin and dx == 0 and dy == 0:
      #getNearestFace(doSaveImage=True)
      okCount += 1
      if okCount > 1:
        if num < 0:
          speak('Hello')
          okCount = 0 
        else:
         break

def introduction1():
  stat = OpenHRP.RobotHardwareServicePackage.RobotStateHolder()
  sample.rh_svc.getStatus(stat)
  pan  = stat.value.command[1]*180.0/math.pi
  tilt0 = stat.value.command[2]*180.0/math.pi
  speak('hello, nice to see you.')
  tilt0 = min(tilt0,  40)
  sample.setJointAnglesDeg([[0,pan, tilt0+30],[],[],[],[]], 1, wait=True)
  sample.setJointAnglesDeg([[0,pan, tilt0],[],[],[],[]], 1, wait=True)
  speak('My Name is Hee row.')
  time.sleep(1)
  speak('I use open are tea em inside.')
  time.sleep(2)
  speak('Welcome to our booth.')
  time.sleep(1)
  speak("I'm demonstrating open sea vee.")
  time.sleep(2)


#########1#########2#########3#########4#########5#########6#########7#########8
#
#  Main
#
if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None

  conf = DemoConfig()

  speak('initializing system')
  init(robotHost)

  while(1):
    sample.rh_svc.servo('LHAND', sample.SwitchStatus.SWITCH_OFF)
    sample.rh_svc.servo('RHAND', sample.SwitchStatus.SWITCH_OFF)
    time.sleep(1)
    sample.servoOn(doConfirm=False) 
    time.sleep(2)
    sample.servoOn(doConfirm=False) 
    speak('servo on')
    sample.rh_svc.servo('LHAND', sample.SwitchStatus.SWITCH_ON)

    speak('moving to initial pose, for demonstration.')
    sample.lhandOpen(30)
    sample.rhandOpen(30)
    if bodyinfo.modelName == 'PARM':
      sample.goInitial()
    elif bodyinfo.modelName == 'HIRONX':
      sample.setJointAnglesDeg([[0, 0, 55],
                            [-16,-19,-130,-43, 46, 0], 
                            [-16, -20.0, -97, -17, 31, 7.6],
                            [], 
                            []],
                            5)
    moveTray('shuffle')
    loop(1)   
    moveTray('push')
    speak('finished')
    sample.goOffPose(wait=False)
    sample.lhandClose()
    sample.rhandClose()
    mask = 31
    wait = True
    sample.seq_svc.isEmpty(mask, wait)
    speak('servo off')
    sample.servoOff(doConfirm=False)
    speak('Have a nice day!')

    if vs_head != None:
      sample.servoOn('BODY',doConfirm=False) 
      time.sleep(1)
      sample.servoOn('BODY',doConfirm=False) 
      loopDetectFace(100)
      #loopDetectFace(-1)
      introduction1()
    else:
      for i in range(1):
        time.sleep(SLEEP_ONE_BLOCK)
        if i == 0:
          print 'waited 1 minute'
        else:
          print 'waited %d minutes'%(i+1)
      time.sleep(10)
      speak('fully rested. run demo?')
    #raw_input('run demo? >')
