#!/opt/grx/bin/hrpsyspy

import sys
import time
import rtm
import Img.CameraCaptureServiceHelper
import OpenHRP.CvProcessorServiceHelper
import java.lang.System

def init(host='localhost'):
  global vs, vs_svc, cvp, cvp_svc
  if robotHost != None:
    print 'robot host = '+robotHost
    java.lang.System.setProperty('NS_OPT',
        '-ORBInitRef NameService=corbaloc:iiop:'+robotHost+':2809/NameService')
    rtm.initCORBA()

  vs = rtm.findRTC("VideoStream0")
  vs_svc = Img.CameraCaptureServiceHelper.narrow(vs.service('service0'))
  vs.start()

  cvp = rtm.findRTC("CvProcessor0")
  cvp_svc = OpenHRP.CvProcessorServiceHelper.narrow(cvp.service('service0'))
  cvp.start()

  rtm.connectPorts(vs.port("MultiCameraImages"),   cvp.port("MultiCameraImage"))

def loop():
  vs_svc.take_one_frame()
  time.sleep(2)
  while 1:
    vs_svc.take_one_frame()
    time.sleep(0.5)

if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None
  init(robotHost)
  cvp.ref.get_configuration().activate_configuration_set('orange')
  loop()   

