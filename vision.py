#!/opt/grx/bin/hrpsyspy

import sys
import time
import rtm
import sample
import Img.CameraCaptureServiceHelper
import OpenHRP.CvProcessorServiceHelper
import java.lang.System

x_upper_limit =  0.6
x_lower_limit =  0

yL_upper_limit = 0.5
yL_lower_limit =-0.1

z_upper_limit  = 0.3
z_lower_limit  = 0.1

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

def loop():
  count = 0
  while 1:
    time.sleep(1)
    x,y,z,roll,pitch,yaw = sample.getCurrentConfiguration(sample.armL_svc)
    print "\n( x, y, z) = %6.3f,%6.3f,%6.3f  unit:[m]"%(x, y, z)

    cx, cy, r = getCircles()
    if r > 0.00001: # circle founced
      cx = -cx + 0.5
      cy = -cy + 0.5
      dx = dy = dz = 0.0

      # determin the control values
      if abs(cx)   > 0.05:
        dx = 0.01
      elif abs(cx) > 0.02:
        dx = 0.002
      if cx < 0:
        dx *= -1

      if abs(cy)   > 0.05:
        dy = 0.01
      elif abs(cy) > 0.02:
        dy = 0.002
      if cy < 0:
        dy *= -1

      if   r < 0.12:
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
        sample.moveRelativeL(dx= 0.000, dy= 0.000, dz= 0.065, rate=10)
        sample.moveRelativeL(dx= 0.000, dy= 0.100, dz= 0.000, rate=10)
        sample.lhandOpen60()
        sample.moveRelativeL(dx=-0.035, dy=-0.100, dz= 0.0,   rate=10)
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
      
      if x < 0.4:
        dx = 0.02

      if 0.05 < y:
        dy = -0.02
      elif y < -0.05:
        dy = 0.02

      if z < z_upper_limit:
        dz = 0.02
    
    print "(dx,dy,dz) = %6.3f,%6.3f,%6.3f  unit:[m]"%(dx, dy, dz)
    if x+dx<x_lower_limit or x_upper_limit<x+dx:
      dx = 0
    if y+dy<yL_lower_limit or yL_upper_limit<y+dy:
      dy = 0
    if z+dz<z_lower_limit or z_upper_limit<z+dz:
      dz = 0
    if not sample.moveL(x+dx, y+dy, z+dz,0,-1.57075,0):
      print "ik error."

if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None
  init(robotHost)
  cvp.ref.get_configuration().activate_configuration_set('orange')
  loop()   
