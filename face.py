#!/opt/grx/bin/hrpsyspy
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

SimulationRun = False

x_upper_limit =  0.6
x_lower_limit =  0.3

yL_upper_limit = 0.3
yL_lower_limit =-0.05

z_upper_limit  = 0.06+0.118
#z_lower_limit  = 0.11+0.065
z_lower_limit  = 0.06+0.065 - 0.065 # TODO

ABS_Z_TO_PICK_TRAY = 0.084
POSITION_X_LIMIT = 0.375

ik_errorlog_filename = '/tmp/ikerror'

#rate70=70
rate70=60
rate60=60
rate50=50
rate40=40
rate30=30
rate20=20
rate10=10
rateDefault=rate30
rate5=5

def speak(s):
  import os
  if type(s) == types.StringType:
    os.system('python speak.py "'+s+'"')
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
  vs = rtm.findRTC("VideoStream0")
  vs_svc = Img.CameraCaptureServiceHelper.narrow(vs.service('service0'))
  vs.start()

  cvp = rtm.findRTC("CvProcessor0")
  cvp_svc = OpenHRP.CvProcessorServiceHelper.narrow(cvp.service('service0'))
  cvp.start()

  rtm.connectPorts(vs.port("MultiCameraImages"),   cvp.port("MultiCameraImage"))
  vs_svc.take_one_frame()

  # for head
  vs_head = rtm.findRTC("VideoStream0_head")
  vs_head_svc = Img.CameraCaptureServiceHelper.narrow(vs_head.service('service0'))
  vs_head.start()

  cvp_head = rtm.findRTC("CvProcessor0_head")
  cvp_head_svc = OpenHRP.CvProcessorServiceHelper.narrow(cvp_head.service('service0'))
  cvp_head.start()

  rtm.connectPorts(vs_head.port("MultiCameraImages"),   cvp_head.port("MultiCameraImage"))
  vs_head_svc.take_one_frame()

  time.sleep(1)

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
    x /= success_count*480.0
    y /= success_count*640.0
    r /= success_count*640.0

  return x, y, r


def pickBall(dropDy):
  x0,y0,z0,roll0,pitch0,yaw0 = \
      sample.getCurrentConfiguration(sample.armL_svc)
  dzz = z_lower_limit - z0
  if sample.moveRelativeL(dx= 0.035, dz=dzz, rate=rate10) < 0:
    print "ik error."
    speak('eye kay error.')
    speak('pull the tray.')
    moveTray('pull')
    return False
  #sample.lhandOpen30()
  for i in range(5):
    sample.lhandOpen(50-i*5)
    time.sleep(0.25)
  sample.moveRelativeL(dz=-dzz, rate=rate70) # rate = 10
  sample.moveRelativeL(dy= dropDy, rate=rate60) # rate = 10 
  sample.lhandOpen60()
  time.sleep(0.3)
  sample.moveRelativeL(dx=-0.035, dy=-dropDy, rate=rate60)
  return True

def moveTray(mode = 'shuffle'):
  global POSITION_X_LIMIT
  # rotate
  sample.moveRelativeL(dy=0.035, dw=-1.57075, rate=rate40)
  
  # move to z_upper_limit
  x0,y0,z0,roll0,pitch0,yaw0 = sample.getCurrentConfiguration(sample.armL_svc)
  lines   = OpenHRP.iarray4SeqHolder()
  print "\n( x, y, z, r, p, w) = %6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f  unit:[m]"%(x0, y0, z0, roll0, pitch0, yaw0)
  sample.moveL(x0, y0, z_upper_limit, roll0, pitch0, yaw0, rate=rateDefault)

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
      comX = comX / 2.0 / len(lines.value) / 640.0 - 0.5 + 0.01
      comY = comY / 2.0 / len(lines.value) / 480.0 - 0.5 
  
      # determine the control values
      if abs(comX)   > 0.05:
        dx = 0.01
      elif abs(comX) > 0.03: # used to be 0.02 
        dx = 0.002
      if comX > 0:
        dx *= -1

      if abs(comY)   > 0.05:
        dy = 0.01
      elif abs(comY) > 0.03: # used to be 0.02 
        dy = 0.002
      if comY < 0:
        dy *= -1
      
      if dx == 0 and dy == 0:
        okCount += 1
        if okCount > 3:
          speak('handle detected.')
          break

      print "x[0]=%6.3f comX=%6.3f dx=%6.3f y[0]=%6.3f comY=%6.3f dy=%6.3f"%(lines.value[0][0], comX, dx, lines.value[0][0], comY, dy)
    else:
      speak('detecting handle.')
      time.sleep(1)
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
      #if len(lines.value) == 0:
      searchDirec *= -1
    sample.moveRelativeL(dx=dx, dy=dy, rate=rate50)
  #  open & down
  sample.lhandOpen60()
  time.sleep(0.3)
  dzz = ABS_Z_TO_PICK_TRAY - z1 # TODO 
  sample.moveRelativeL(dy=-0.035, dz=dzz, rate=rate10)

  # grasp & do the action
  sample.lhandClose()
  time.sleep(0.3)
  if mode == 'pull':
    #sample.moveRelativeL(dx=-0.05, rate=10)
    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    sample.moveL(0.35,y1,z1,roll1,pitch1,yaw1,rate=rate20)
  elif mode == 'shuffle':
    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    sample.moveL(0.35,y1,z1,roll1,pitch1,yaw1,rate=rateDefault)

    sample.moveRelativeL(dx= 0.015, dz= 0.06, rate=rate5)
    time.sleep(2)
    sample.moveRelativeL(dx=-0.015, dz=-0.06, rate=rate5)
  elif mode == 'push':
    x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
    sample.moveL(0.42,y1,z1,roll1,pitch1,yaw1, rate=rate20)

  # open & up & rotate
  #sample.lhandOpen60()
  sample.lhandOpen()
  time.sleep(0.3)
  x1,y1,z1,roll1,pitch1,yaw1 = sample.getCurrentConfiguration(sample.armL_svc)
  #sample.moveRelativeL(dz=0.1, rate=20)
  sample.moveL(x1, y1, z_upper_limit, roll1, pitch1, yaw1, rate=rate20)
  sample.moveRelativeL(dx=0.035,dw=1.57075, rate=rate40)

def loop(numTry=1):
  count = 0
  shuffleCount = 0
  sample.lhandOpen60()
  lastY = 0
  tryCount = 0
  numOrange = 0
  numBlue = 0

  x0,y0,z0,roll0,pitch0,yaw0 = sample.getCurrentConfiguration(sample.armL_svc)
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

    if r > 0.05: # circle found
      print detectColor
      shuffleCount = 0
      cx = -cx + 0.5
      cy = -cy + 0.5
      dx = dy = dz = 0.0

      # determine the control values
      if abs(cx)   > 0.05:
        dx = 0.01
      elif abs(cx) > 0.03: # used to be 0.02 
        dx = 0.002
      if cx < 0:
        dx *= -1

      if abs(cy)   > 0.05:
        dy = 0.01
      elif abs(cy) > 0.03: # used to be 0.02
        dy = 0.002
      if cy < 0:
        dy *= -1

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
      if r > 0.00001: # circle found
        speak('ignored tiny circle')
        time.sleep(3)
      # circle NOT founded
      count = 0;
      dx = dy = dz = 0
      
      #if x < 0.4:
      #  dx = 0.02

      #if lastY + 0.03 < y:
      #  dy = -0.02
      #elif y < lastY - 0.03:
      #  dy = 0.02

      if z + 0.02 < z_upper_limit:
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
    #if (x+dx<x_lower_limit and dx<0) or (x_upper_limit<x+dx and 0<dx):
    if (x+dx<POSITION_X_LIMIT and dx<0) or (x_upper_limit<x+dx and 0<dx):
      print "x limit"
      #speak('limited ecks movement')
      speak('return to initial position')
      rate = rate10
      dx = x0 - x
      dy = y0 - y
      dz = z0 - z
    else:
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

    #sample.moveL(x+dx, y+dy, z+dz,0,-1.57075,0, rate=rateDefault) # wrong?
    if sample.moveL(x+dx, y+dy, z+dz,0,-1.57075,0, rate=rateDefault) < 0:
      print "ik error."
      speak('eye kay error.')
      logIkErrorPos(x+dx, y+dy, z+dz,0,-1.57075,0)

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
    cvp_head_svc.detectFaces(ret, doSaveImage)
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
    x /= success_count*480.0
    y /= success_count*640.0
    r /= success_count*640.0

  return x, y, r

def loopDetectFace(num):
  sample.setJointAnglesDeg([[0,0, 10],[],[],[],[]], 0.5, wait=True)
  okCount = 0
  time.sleep(1)
  for i in range(num):
    cx,cy,r = getNearestFace()
    if r < 0.05:
      time.sleep(1)
      continue
    cx = -cx + 0.5
    cy = -cy + 0.5

    # determine the control values
    dx = dy = 0
    if abs(cx)   > 0.15:
      dx = 0.01
    elif abs(cx) > 0.10: # used to be 0.02 
      dx = 0.002
    #if cx < 0:
    if cx > 0:
      dx *= -1

    if abs(cy)   > 0.15:
      dy = 0.01
    elif abs(cy) > 0.10: # used to be 0.02
      dy = 0.002
    if cy < 0:
      dy *= -1

    print "dx,dy = %f %f"%(dx, dy)
    stat = OpenHRP.RobotHardwareServicePackage.RobotStateHolder()
    sample.rh_svc.getStatus(stat)
    pan  = stat.value.command[1]*180.0/math.pi + dx * 100
    tilt = stat.value.command[2]*180.0/math.pi + dy * 100
    pan = min(pan,  70);
    pan = max(pan, -70);
    tilt = min(tilt,   70);
    tilt = max(tilt,   10);
    print "pan, tilt : %f %f" %(pan, tilt)
    sample.setJointAnglesDeg([[0,pan, tilt],[],[],[],[]], 0.4, wait=True)

    if dx == 0 and dy == 0:
      #getNearestFace(doSaveImage=True)
      okCount += 1
      if okCount > 1:
        speak('Hello guys')
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


if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None

  speak('initializing system')
  init(robotHost)

  while(1):
    sample.servoOn('BODY',doConfirm=False) 
    time.sleep(1)
    sample.servoOn('BODY',doConfirm=False) 

    loopDetectFace(100)
    introduction1()

    speak('servo on')
    sample.servoOn(doConfirm=False) 
    time.sleep(2)
    sample.servoOn(doConfirm=False) 
    time.sleep(1)
    sample.servoOn('LHAND', doConfirm=False) 
    speak('moving to initial pose, for demonstration.')
    sample.lhandOpen(30)
    sample.rhandOpen(30)
    if bodyinfo.modelName == 'PARM':
      sample.goInitial()
    else:
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
    #time.sleep(610)
    #for i in range(10):

    #for i in range(1):
    #  time.sleep(60)
    #  if i == 0:
    #    print 'waited 1 minute'
    #  else:
    #    print 'waited %d minutes'%(i+1)
    #time.sleep(10)
    #speak('fully rested. run demo?')
    #raw_input('run demo? >')