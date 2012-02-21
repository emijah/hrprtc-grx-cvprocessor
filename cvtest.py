#!/opt/grx/bin/hrpsyspy

import sys
import time
import rtm
import Img.CameraCaptureServiceHelper
import OpenHRP.CvProcessorServiceHelper
import java.lang.System

def init(host='localhost'):
  global vs0, vs0_svc, vs1, vs1_svc, cvp, cvp_svc
  if robotHost != None:
    print 'robot host = '+robotHost
    java.lang.System.setProperty('NS_OPT',
        '-ORBInitRef NameService=corbaloc:iiop:'+robotHost+':2809/NameService')
    rtm.initCORBA()

  cvp = rtm.findRTC("CvProcessor0_head")
  print 'cvp = ', cvp
  cvp_svc = OpenHRP.CvProcessorServiceHelper.narrow(cvp.service('service0'))
  cvp.start()

  vs0 = rtm.findRTC("VideoStream0_head")
  print 'vs0 = ', vs0
  vs0_svc = Img.CameraCaptureServiceHelper.narrow(vs0.service('service0'))
  vs0.start()
  rtm.connectPorts(vs0.port("MultiCameraImages"),   cvp.port("MultiCameraImage"))

  vs1 = rtm.findRTC("VideoStream0_hand")
  if vs1 != None:
    vs1_svc = Img.CameraCaptureServiceHelper.narrow(vs1.service('service0'))
    vs1.start()
    rtm.connectPorts(vs1.port("MultiCameraImages"),   cvp.port("MultiCameraImage"))


def loop():
  vs0_svc.take_one_frame()
  time.sleep(2)
  circles = OpenHRP.darray3SeqHolder()
  lines   = OpenHRP.iarray4SeqHolder()
  while 1:
    vs0_svc.take_one_frame()
    time.sleep(1)
    if 1:
      cvp_svc.HoughCircles(0, circles)
    else:
      cvp_svc.HoughLinesP(0, lines)
    if vs1 != None:
      vs1_svc.take_one_frame()
      time.sleep(1)
      cvp_svc.HoughCircles(ret)

if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None
  init(robotHost)
  loop()   

