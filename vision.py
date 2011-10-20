#!/opt/grx/bin/hrpsyspy

import sys
import time
import rtm
import sample
import Img.CameraCaptureServiceHelper
import OpenHRP.CvProcessorServiceHelper
import java.lang.System

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

def getCircles():
  NUM_TO_TAKE=1
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
      x += ret.value[maxidx][1]
      y += ret.value[maxidx][0]
      r += ret.value[maxidx][2]
  if success_count > 0:
    x /= success_count*480.0
    y /= success_count*640.0
    r /= success_count*640.0

  return x, y, r

def loop():
  count = 0
  while 1:
    time.sleep(1)
    x,y,z,r,p,w = sample.getCurrentConfiguration(sample.armL_svc)
    print "( x, y, z) = %6.3f,%6.3f,%6.3f  unit:[m]"%(x, y, z)

    cx, cy, r = getCircles()
    if r > 0.00001: # circle founced
      cx = -cx + 0.5
      cy = -cy + 0.5

      dx = dy = dz = 0.0

      if abs(cx)   > 0.1:
        dx = 0.01
      elif abs(cx) > 0.02:
        dx = 0.002
      if cx > 0:
        dx *= -1

      if abs(cy)   > 0.1:
        dy = 0.01
      elif abs(cy) > 0.02:
        dy = 0.002
      if cy > 0:
        dy *= -1

      if z < 0.1:
        dz =  0.0
      elif r < 0.1:
        dz = -0.01
      elif r < 0.15:
        dz = -0.003

      if dx == 0 and dy == 0 and dz == 0:
        count += 1
      elif count > 1:
        count -= 1

      if count > 2:
        # grasping
	lastPoint = [x, y, z]
        sample.moveRelativeL(dx= 0.035, dy= 0.000, dz=-0.065, rate=10)
        sample.lhandOpen30()
        sample.moveRelativeL(dx= 0.000, dy= 0.100, dz= 0.065, rate=10)
        sample.lhandOpen60()
        sample.moveRelativeL(dx=-0.035, dy=-0.100, dz= 0.0,   rate=10)
        count = 0;
      else:
        #
        print "(cx,cy, r) = %6.3f,%6.3f,%6.3f  unit:-"%(cx, cy, r)
        print "(dx,dy,dz) = %6.3f,%6.3f,%6.3f  unit:[m]"%(dx, dy, dz)
	print "count="+str(count)
        sample.moveRelativeL(dx=dx/(count+1.0), dy=dy/(count+1.0), dz=dz)
    else: 
      # circle NOT founded
      count = 0;
      dx = dy = dz = 0
      
      if x < 0.4:
        dx = 0.02

      if 0.05 < y:
        dy = -0.02
      elif y < -0.05:
        dy = 0.02

      if z < 0.3:
        dz = 0.02

      sample.moveRelativeL(dx=dx, dy=dy, dz=dz)

if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None
  init(robotHost)
  cvp.ref.get_configuration().activate_configuration_set('orange')
  loop()   
