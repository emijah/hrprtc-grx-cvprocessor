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

  cvp = rtm.findRTC("CvProcessor0")
  cvp_svc = OpenHRP.CvProcessorServiceHelper.narrow(cvp.service('service0'))
  cvp.start()

  vs0 = rtm.findRTC("VideoStream0")
  vs0_svc = Img.CameraCaptureServiceHelper.narrow(vs0.service('service0'))
  vs0.start()
  rtm.connectPorts(vs0.port("MultiCameraImages"),   cvp.port("MultiCameraImage"))

  vs1 = rtm.findRTC("VideoStream1")
  if vs1 != None:
    vs1_svc = Img.CameraCaptureServiceHelper.narrow(vs1.service('service0'))
    vs1.start()
    rtm.connectPorts(vs1.port("MultiCameraImages"),   cvp.port("MultiCameraImage"))


def loop():
  ret = OpenHRP.darray3SeqHolder()
  iret = OpenHRP.iarray4SeqHolder()
  vs0_svc.take_one_frame()
  time.sleep(2)
  while 1:
    vs0_svc.take_one_frame()
    time.sleep(2)
    #cvp_svc.HoughCircles(ret)
    cvp_svc.HoughLinesP(iret)
    if vs1 != None:
      vs1_svc.take_one_frame()
      time.sleep(2)
      cvp_svc.HoughCircles(ret)

if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None
  init(robotHost)
  cvp.ref.get_configuration().activate_configuration_set('orange')
  loop()   

