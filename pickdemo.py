#!/opt/grx/bin/hrpsyspy

import sys
import time
import rtm
import sample
import types
import Img.CameraCaptureServiceHelper
import OpenHRP.CvProcessorServiceHelper
import java.lang.System

#############################
#
# Parameters
#
x_upper_limit =  0.2
x_lower_limit =  0.3

yL_upper_limit = 0.1
yL_lower_limit =-0.05

z_upper_limit  = 0.06+0.118
#z_lower_limit  = 0.11+0.065
z_lower_limit  = 0.06+0.065

# number of pixels
NUM_PIXELS_X=480
NUM_PIXELS_Y=640

# the distance between camera center and hand center
#CAMERA_OFFSET_X=0.035 #for HIRO
CAMERA_OFFSET_X=0.0    #for PARM

# the z displacement to pick ball in absolute value [m]
DISPLACEMENT_Z_TO_PICK_BALL=0.069

# the z displacement to pick tray in absolute value [m]
DISPLACEMENT_Z_TO_PICK_TRAY = 0.100

# the x,z displacement to shuffle tray in absolute value [m]
DISPLACEMENT_X_TO_SHUFFLE_TRAY = 0.015
DISPLACEMENT_Z_TO_SHUFFLE_TRAY = 0.060

# the absolute position x to push/pull tray [m]
POSITION_X_TO_PULL_TRAY = 0.35
POSITION_X_TO_PUSH_TRAY = 0.42

# contol value x for 5% diffence [m]
CONTROL_VALUE_X_05 = 0.01
# contol value x for 3% diffence [m]
CONTROL_VALUE_X_03 = 0.002

# contol value y for 5% diffence [m]
CONTROL_VALUE_Y_05 = 0.002
# contol value y for 3% diffence [m]
CONTROL_VALUE_Y_03 = 0.01

# contol value z to search the target
CONTROL_VALUE_Z_SEARCHING = 0.02
#############################

#############################
#
# Local Log File
#
ik_errorlog_filename = '/tmp/ikerror'

def speak(s):
  import os
  if type(s) == types.StringType:
    os.system('spd-say "'+s+'"')
  else:
    print 'error(speak): input is not a string'

def logIkErrorPos(x,y,z,r,p,w):
  global ik_errorlog_filename
  of = open(ik_errorlog_filename, 'a')
  of.write('%f,%f,%f,%f,%f,%f\n'%(x,y,z,r,p,w))
  of.close()

def init(host='localhost'):
  global vs, vs_svc, cvp, cvp_svc
  sample.init(host)
  vs = rtm.findRTC("VideoStream0")
  vs_svc = Img.CameraCaptureServiceHelper.narrow(vs.service('service0'))
  vs.start()

  cvp = rtm.findRTC("CvProcessor0")
  cvp_svc = OpenHRP.CvProcessorServiceHelper.narrow(cvp.service('service0'))
  cvp.start()

  rtm.connectPorts(vs.port("MultiCameraImages"),   cvp.port("MultiCameraImage"))
  vs_svc.take_one_frame()
  time.sleep(1)


def getControlValue():
  # determine the control values
  if abs(comX)   > 0.05:
    dx = CONTROL_VALUE_X_05 
  elif abs(comX) > 0.03: # used to be 0.02 
    dx = CONTROL_VALUE_X_03
  if comX > 0:
    dx *= -1

  if abs(comY)   > 0.05:
    dy = CONTROL_VALUE_Y_05 
  elif abs(comY) > 0.03: # used to be 0.02 
    dy = CONTROL_VALUE_Y_03
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
    cvp_svc.HoughCircles(ret)
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
    x /= float(success_count*NUM_PIXELS_X)
    y /= float(success_count*NUM_PIXELS_X)
    r /= float(success_count*NUM_PIXELS_X)

  return x, y, r


def pickBall(dropDy):
  # hard coded coordinates
  if sample.moveRelativeL(dx=CAMERA_OFFSET_X, dz=-DISPLACEMENT_Z_TO_PICK_BALL, rate=10) < 0:
    print "ik error."
    speak('eye kay error.')
    speak('pull the tray.')
    moveTray('pull')
    return False
  #sample.lhandOpen30()
  for i in range(5):
    sample.lhandOpen(50-i*5)
    time.sleep(0.25)
  # hard coded coordinates
  sample.moveRelativeL(dz= DISPLACEMENT_Z_TO_PICK_BALL, rate=70) # rate = 10
  # hard coded coordinates
  sample.moveRelativeL(dy= dropDy, rate=60) # rate = 10 
  sample.lhandOpen60()
  time.sleep(0.3)
  # hard coded coordinates
  sample.moveRelativeL(dx=-CAMERA_OFFSET_X, dy=-dropDy, rate=60)
  return True

def moveTray(mode = 'shuffle'):
  # rotate
  # hard coded coordinates
  sample.moveRelativeL(dy=CAMERA_OFFSET_X, dw=-1.57075, rate=40)
  
  # move to z_upper_limit
  x0,y0,z0,roll0,pitch0,yaw0 = sample.getCurrentConfiguration(sample.armL_svc)
  lines   = OpenHRP.iarray4SeqHolder()
  print "\n( x, y, z, r, p, w) = %6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f  unit:[m]"%(x0, y0, z0, roll0, pitch0, yaw0)
  sample.moveL(x0, y0, z_upper_limit, roll0, pitch0, yaw0)

  # detect handle
  okCount = 0
  searchDirec = -1
  while 1:
    cvp.ref.get_configuration().activate_configuration_set('green')
    vs_svc.take_one_frame()
    time.sleep(0.1)
    cvp_svc.HoughLinesP(lines)
    dx = dy = 0.0
    if len(lines.value) > 0:
      ditectDelec = 0
      comX = comY = 0.0
      for pnt in lines.value:
        comX += pnt[0] + pnt[2]
        comY += pnt[1] + pnt[3]
      comX = comX / 2.0 / len(lines.value) / NUM_PIXELS_Y - 0.5 + 0.01
      comY = comY / 2.0 / len(lines.value) / NUM_PIXELS_X - 0.5 
 
      dx,dy = getControlValue(comX, comY)
      
      if dx == 0 and dy == 0:
        okCount += 1
        if okCount > 3:
          speak('handle detected.')
          break

      print "x[0]=%6.3f comX=%6.3f dx=%6.3f y[0]=%6.3f comY=%6.3f dy=%6.3f"%(lines.value[0][0],
                                                                             comX,
                                                                             dx,
                                                                             lines.value[0][0],
                                                                             comY,
                                                                             dy)
    else:
      speak('detecting handle.')
      dy = searchDirec * 0.03
      okCount = 0

    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    if (x1+dx<x_lower_limit and dx<0) or (x_upper_limit<x1+dx and 0<dx):
      print "x limit"
      speak('limited ecks movement')
      dx = 0
    if (y1+dy<yL_lower_limit and dy<0) or (yL_upper_limit<y1+dy and 0<dy):
      print "y limit"
      speak('limited wai movement')
      dy = 0
      if len(lines.value) == 0:
        searchDirec *= -1
    sample.moveRelativeL(dx=dx, dy=dy, rate=50)
  #  open & down
  sample.lhandOpen60()
  time.sleep(0.3)
  # hard coded coordinates(typo)
  sample.moveRelativeL(dy=-CAMERA_OFFSET_X, dz=-DISPLACEMENT_Z_TO_PICK_TRAY, rate=10)

  # grasp & do the action
  sample.lhandClose()
  time.sleep(0.3)
  if mode == 'pull':
    #sample.moveRelativeL(dx=-0.05, rate=10)
    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    sample.moveL(POSITION_X_TO_PULL_TRAY,y1,z1,roll1,pitch1,yaw1)
  elif mode == 'shuffle':
    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    sample.moveL(POSITION_X_TO_PULL_TRAY,y1,z1,roll1,pitch1,yaw1)

    # hard coded coordinates
    sample.moveRelativeL(dx= DISPLACEMENT_Z_TO_SHUFFLE_TRAY, dz= DISPLACEMENT_Z_TO_SHUFFLE_TRAY, rate=5)
    time.sleep(2)
    # hard coded coordinates
    sample.moveRelativeL(dx=-DISPLACEMENT_Z_TO_SHUFFLE_TRAY, dz=-DISPLACEMENT_Z_TO_SHUFFLE_TRAY, rate=5)
  elif mode == 'push':
    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    sample.moveL(POSITION_X_TO_PUSH_TRAY,y1,z1,roll1,pitch1,yaw1)

  # open & up & rotate
  sample.lhandOpen60()
  time.sleep(0.3)
  # hard coded coordinates
  sample.moveRelativeL(dz=DISPLACEMENT_Z_TO_PICK_TRAY, rate=20)
  # hard coded coordinates
  sample.moveRelativeL(dx=CAMERA_OFFSET_X,dw=1.57075, rate=40)

def loop(numTry=1):
  count = 0
  shuffleCount = 0
  sample.lhandOpen60()
  lastY = 0
  tryCount = 0
  numOrange = 0
  numBlue = 0
  while 1:
    x,y,z,roll,pitch,yaw = sample.getCurrentConfiguration(sample.armL_svc)
    print "\n( x, y, z, r, p, w) = %6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f  unit:[m]"%(x, y, z, roll, pitch, yaw)

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

    if r > 0.00001: # circle found
      print detectColor
      shuffleCount = 0
      cx = -cx + 0.5
      cy = -cy + 0.5
      dx = dy = dz = 0.0

      # determine the control values
      dx,dy = getControlValue(cx,cy)

      #if   r < 0.12:
      #  dz = -0.01
      #elif r < 0.15:
        #dz = -0.003
      if r < 0.15:
        dz = -0.01
      #      #  dz =  0.003

      # speak adjusting parameters

      if dx == 0 and dy == 0 and (dz <= z_lower_limit or dz == 0):
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
      # circle NOT founded
      count = 0;
      dx = dy = dz = 0
      
      #if x < 0.4:
      #  dx = 0.02

      #if lastY + 0.03 < y:
      #  dy = -0.02
      #elif y < lastY - 0.03:
      #  dy = 0.02

      # hard coded coordinates
      if z + CONTROL_VALUE_Z_SEARCHING < z_upper_limit:
        dz = CONTROL_VALUE_Z_SEARCHING

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
      
    print "(dx,dy,dz) = %6.3f,%6.3f,%6.3f  unit:[m]"%(dx, dy, dz)
    if (x+dx<x_lower_limit and dx<0) or (x_upper_limit<x+dx and 0<dx):
      print "x limit"
      speak('limited ecks movement')
      dx = 0
    if (y+dy<yL_lower_limit and dy<0) or (yL_upper_limit<y+dy and 0<dy):
      print "y limit"
      speak('limited wai movement')
      dy = 0
    if z+dz<z_lower_limit and dz<0:
      print "z lower limit"
      #speak('limited zed movement')
      dz = z_lower_limit - z
    elif z_upper_limit<z+dz and 0<dz:
      print "z upper limit"
      dz = z_upper_limit - z

    sample.moveL(x+dx, y+dy, z+dz,0,-1.57075,0)
    if sample.moveL(x+dx, y+dy, z+dz,0,-1.57075,0) < 0:
      print "ik error."
      speak('eye kay error.')
      logIkErrorPos(x+dx, y+dy, z+dz,0,-1.57075,0)


if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None

  speak('initializing system')
  init(robotHost)
  speak('servo on')
  sample.servoOn(doConfirm=False) 
  speak('moving to the initial pose for demonstration.')
  sample.goInitial()
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
