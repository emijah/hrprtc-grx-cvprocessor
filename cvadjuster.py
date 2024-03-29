#!/opt/grx/bin/hrpsyspy
import sys
import time
import rtm
import Img.CameraCaptureServiceHelper
import OpenHRP.CvProcessorServiceHelper
import java.lang.System

from java.awt import *
from javax.swing import *
from javax.swing.event import *
from javax.swing.border import *

from _SDOPackage import *

mode = 'circle'
defaultProp = {}
def loadDefaultValue(fname):
  global defaultProp
  for l in open(fname):
    if l.strip() == '' or l.strip().startswith('#'):
      continue
    toks = l.strip().split("=")
    key = toks[0].strip().split('.')
    if not defaultProp.has_key(key[0]):
      defaultProp[key[0]] = {}
    defaultProp[key[0]][key[1]] = toks[1]
    print key[0] + "-" + key[1] + "=" + toks[1]

# Slider Panel for 0 to 256
class SliderPanel( JPanel, ChangeListener ):

  def __init__( self, rtc, conf_set_name, param_name, inival=128, minval=0, maxval=256 ):
    bdcolor = Color( 32, 32, 32, 32 )
    lborder = LineBorder( Color( 32, 32, 32, 32 ), 1, False )
    
    self.rtc = rtc
    self.conf_set_name = conf_set_name
    self.param_name = param_name
    
    self.defaultValue = inival
    self.layout = BoxLayout( self, BoxLayout.X_AXIS )
    self.border = TitledBorder( lborder, param_name )
    
    self.valtext = JLabel()
    self.valtext.setPreferredSize( Dimension( 30, 30 ) )
    self.valtext.setHorizontalAlignment( JLabel.RIGHT )
    self.valtext.setAlignmentY( 1.0 )
    self.add( self.valtext )
    
    self.slider = JSlider( minval, maxval, inival )
    self.slider.setPaintLabels( True )
    self.slider.setMinorTickSpacing( 16 )
    self.slider.setMajorTickSpacing( 64 )
    self.slider.setPaintTicks( True )
    self.slider.setLabelTable( self.slider.createStandardLabels( 128 ) )
    self.slider.setFont( Font( Font.SANS_SERIF, Font.BOLD, 10 ) )
    self.slider.addChangeListener( self )
    
    self.add( self.slider )
    
    self.valtext.setText( str( self.getSliderValue() ) )

    self.button = JButton('d', actionPerformed=self.setDefaultValue)
    self.add( self.button )

  def setDefaultValue(self, e):
    self.setSliderValue(self.defaultValue)  
    self.stateChanged(None)

  def setSliderValue( self, value ):
    self.slider.setValue( value )
    self.valtext.setText( str( self.getSliderValue() ) )
  
  def getSliderValue( self ):
    return self.slider.getValue()
  
  def stateChanged( self, e ):
    val_str = str( self.getSliderValue() )
    self.valtext.setText( val_str )
    
    if self.slider.getValueIsAdjusting() == False :
      rtc = self.rtc
      cs_name = self.conf_set_name
      pr_name = self.param_name
      
    #  setRtcConfiguration( rtc, cs_name, pr_name, val_str )
      setRtcConfiguration_activate( rtc, cs_name, pr_name, val_str )
      printRtcConfiguration( rtc )
      print 'Changed -> %s : %s = %s' % ( cs_name, pr_name, val_str )


# Slider Panel for 0 to 1 (Percent 0 to 100)
class SliderPanel_Percent( JPanel, ChangeListener ):

  def __init__( self, rtc, conf_set_name, param_name, inival=0.5, maxval=1 ):
    self.ratio = maxval / 100.0
    self.defaultValue = inival
    inival = int( inival / self.ratio )
    minval = 0
    maxval = maxval * 100
    
    bdcolor = Color( 32, 32, 32, 32 )
    lborder = LineBorder( Color( 32, 32, 32, 32 ), 1, False )
    
    self.rtc = rtc
    self.conf_set_name = conf_set_name
    self.param_name = param_name
    
    self.layout = BoxLayout( self, BoxLayout.X_AXIS )
    self.border = TitledBorder( lborder, param_name + ' (Slider:Percent)' )
    
    self.valtext = JLabel()
    self.valtext.setPreferredSize( Dimension( 30, 30 ) )
    self.valtext.setHorizontalAlignment( JLabel.RIGHT )
    self.valtext.setAlignmentY( 1.0 )
    self.add( self.valtext )
    
    self.slider = JSlider( minval, maxval, inival )
    self.slider.setPaintLabels( True )
    self.slider.setMinorTickSpacing( 10 )
    self.slider.setMajorTickSpacing( 20 )
    self.slider.setPaintTicks( True )
    self.slider.setLabelTable( self.slider.createStandardLabels( 50 ) )
    self.slider.setFont( Font( Font.SANS_SERIF, Font.BOLD, 10 ) )
    self.slider.addChangeListener( self )
    self.add( self.slider )

    self.button = JButton('d', actionPerformed=self.setDefaultValue)
    self.add( self.button )
    
    self.valtext.setText( str( self.getSliderValue() )[:4] )

  def setDefaultValue(self, e):
    self.setSliderValue(self.defaultValue)  
    self.stateChanged(None)
  
  def setSliderValue( self, value ):
    self.slider.setValue( int( value / self.ratio ) )
    self.valtext.setText( str( self.getSliderValue() )[:4] )
  
  def getSliderValue( self ):
    return self.slider.getValue() * self.ratio
  
  def stateChanged( self, e ):
    val_str = str( self.getSliderValue() )
    self.valtext.setText( val_str[:4] )
    
    if self.slider.getValueIsAdjusting() == False :
      rtc = self.rtc
      cs_name = self.conf_set_name
      pr_name = self.param_name
      
    #  setRtcConfiguration( rtc, cs_name, pr_name, val_str )
      setRtcConfiguration_activate( rtc, cs_name, pr_name, val_str )
      printRtcConfiguration( rtc )
      print 'Changed -> %s : %s = %s' % ( cs_name, pr_name, val_str )


def makeConfigurationTabPanel( rtc ) :
  tab_pnl = JTabbedPane()
  
  conf_sets = getRtcConfigurationSets( rtc )
  conf_set = {}
  conf_set_dict = {}
  
  for cs in conf_sets :
    conf_set[cs.id] = cs
    conf_set_dict[cs.id] = nvlist2dict( cs.configuration_data )
    
    pnl_cfgset = JPanel()
    
    pnl_cfgset.layout = BoxLayout( pnl_cfgset, BoxLayout.Y_AXIS )
    tab_pnl.addTab( cs.id, pnl_cfgset )
    
    sorted_keys = conf_set_dict[cs.id].keys()
    sorted_keys.sort()
    for key in sorted_keys :
    #  print '%s : %s = %s' % ( cs.id, key, conf_set_dict[cs.id][key] )
      if defaultProp.has_key(cs.id) and defaultProp[cs.id].has_key(key):
        defaultValue = int( defaultProp[cs.id][key] )
        slider = SliderPanel( rtc, cs.id, key, defaultValue)
	slider.setDefaultValue(None)
      else:
        defaultValue = int( conf_set_dict[cs.id][key]) 
        slider = SliderPanel( rtc, cs.id, key, defaultValue)
      pnl_cfgset.add( slider )
  
  printRtcConfiguration( rtc )
  
  return tab_pnl


def makeConfigurationTabPanel_VideoStream() :
  rtc = vs
  
  tab_pnl = JTabbedPane()
  
  conf_sets = getRtcConfigurationSets( rtc )
  conf_set = {}
  conf_set_dict = {}
  
  for cs in conf_sets :
    conf_set[cs.id] = cs
    conf_set_dict[cs.id] = nvlist2dict( cs.configuration_data )
    
    pnl_cfgset = JPanel()
    
    pnl_cfgset.layout = BoxLayout( pnl_cfgset, BoxLayout.Y_AXIS )
    tab_pnl.addTab( cs.id, pnl_cfgset )
    
    sorted_keys = conf_set_dict[cs.id].keys()
    sorted_keys.sort()
    for key in sorted_keys :
      #if key == 'brightness' :
      #  print '%s : %s = %s' % ( cs.id, key, conf_set_dict[cs.id][key] )
      if defaultProp.has_key(cs.id) and defaultProp[cs.id].has_key(key):
        defaultValue = float( defaultProp[cs.id][key] )
        slider = SliderPanel_Percent( rtc, cs.id, key, defaultValue )
	slider.setDefaultValue(None)
      else:
        defaultValue = float( conf_set_dict[cs.id][key]) 
        slider = SliderPanel_Percent( rtc, cs.id, key, defaultValue )
      pnl_cfgset.add( slider )
  
  printRtcConfiguration( rtc )
  
  return tab_pnl


def init_gui():
  frm = JFrame( "sample", defaultCloseOperation = JFrame.EXIT_ON_CLOSE )
  frm.setAlwaysOnTop( True )
  
  pnl = frm.getContentPane()
  pnl.layout = BoxLayout( pnl, BoxLayout.X_AXIS )
  
  tabpanel1 = makeConfigurationTabPanel( cvp )
  pnl.add( tabpanel1 )
  
  tabpanel2 = makeConfigurationTabPanel_VideoStream()
  pnl.add( tabpanel2 )
  
  frm.pack()
  frm.show()


def init(host='localhost', refSuffix=""):
  global vs, vs_svc, cvp, cvp_svc
  if robotHost != None:
    print 'robot host = '+robotHost
    java.lang.System.setProperty('NS_OPT',
        '-ORBInitRef NameService=corbaloc:iiop:'+robotHost+':2809/NameService')
    rtm.initCORBA()

  if refSuffix == None:
     refSuffix = ''
  elif refSuffix != '':
    refSuffix = '_'+refSuffix
    print 'suffix = '+refSuffix
  vs = rtm.findRTC("VideoStream0"+refSuffix)
  vs_svc = Img.CameraCaptureServiceHelper.narrow(vs.service('service0'))
  vs.start()

  cvp = rtm.findRTC("CvProcessor0"+refSuffix)
  cvp_svc = OpenHRP.CvProcessorServiceHelper.narrow(cvp.service('service0'))
  cvp.start()

  rtm.connectPorts(vs.port("MultiCameraImages"), cvp.port("MultiCameraImage"))


def loop():
  vs_svc.take_one_frame()
  time.sleep(2)
  while 1:
    vs_svc.take_one_frame()
#    time.sleep(0.5)
    time.sleep(1) # ThinkPad X200
    if mode == 'circle':
      circles = OpenHRP.darray3SeqHolder()
      cvp_svc.HoughCircles(0, circles)
      cvp_svc.HoughCircles(1, circles)
    else:
      lines   = OpenHRP.iarray4SeqHolder()
      cvp_svc.HoughLinesP(0, lines)
      cvp_svc.HoughLinesP(1, lines)

#####
# Data Conversion
def nvlist2dict( nvlist ) :
  rslt = {}
  for tmp in nvlist :
  #  rslt[tmp.name]=tmp.value.value()   # nv.value and any.value()
    rslt[tmp.name]=tmp.value.extract_string()   # YY
  return rslt

def dict2nvlist( dict ) :
  rslt = []
  for tmp in dict.keys() :
  #  rslt.append( SDOPackage.NameValue( tmp, any.to_any( dict[tmp] ) ) )
    nv1 = NameValue()
    nv1.name = tmp;
    a1 = rtm.orb.create_any()
    a1.insert_string( dict[tmp] )
    nv1.value = a1
    rslt.append( nv1 )
  return rslt


# Set CVP Configuration and Activate
def setRtcConfiguration( rtc, conf_set_name, param_name, value ) :
  conf_ref = rtc.ref.get_configuration()
  conf_sets = conf_ref.get_configuration_sets()
  if len( conf_sets ) == 0 :
    print "configuration set is not found"
    return
  for cs in conf_sets :
    if cs.id == conf_set_name :
      n_conf_set = cs
      n_conf_set_dict = nvlist2dict( n_conf_set.configuration_data )
      n_conf_set_dict[param_name] = value
      n_conf_set.configuration_data = dict2nvlist( n_conf_set_dict )
      conf_ref.set_configuration_set_values( n_conf_set )
      break;

# Set CVP Configuration and Activate
def setRtcConfiguration_activate( rtc, conf_set_name, param_name, value ) :
  setRtcConfiguration( rtc, conf_set_name, param_name, value )
  conf_ref = rtc.ref.get_configuration()
  conf_ref.activate_configuration_set( conf_set_name )

# Get RTC Configiguration Sets
def getRtcConfigurationSets( rtc ) :
  conf_ref = rtc.ref.get_configuration()
  conf_sets = conf_ref.get_configuration_sets()
  return conf_sets
  
# Print RTC Configurations
def printRtcConfiguration( rtc ) :
  conf_sets = getRtcConfigurationSets( rtc )
  print '\nPrint RTC Configurations:'
  conf_set = {}
  conf_set_dict = {}
  for cs in conf_sets :
    conf_set[cs.id] = cs
    conf_set_dict[cs.id] = nvlist2dict( cs.configuration_data )

    sorted_keys = conf_set_dict[cs.id].keys()
    sorted_keys.sort()
    for key in sorted_keys :
      print '%s : %s = %s' % ( cs.id, key, conf_set_dict[cs.id][key] )
#####

def usage(com):
    print 'Usage:'
    print com + ' --host=<HOSTNAME> --mode=<DETECT_MODE> --config=<CONFIG_FILE> --suffix=<REF_SUFFIX>'
    print '  HOSTNAME = hostname of NameService [localhost, hiro015, etc..]'
    print '  DETECT_MODE = [circle, lines]' 
    print '  CONFIG_FILE = settings of each threshold'
    print '  REF_SUFFIX  = the suffix for rtc reference [ head, hand , etc..]'

import getopt
if __name__ == '__main__' or __name__ == 'main':
  try:
    opts, args = getopt.getopt(sys.argv[1:], 'r:m:c:s:h', ['host=', 'mode=', 'config=', 'suffix=', 'help']) 
  except getopt.GetoptError:
    usage(sys.argv[0])
    sys.exit(2)

  robotHost = None
  refSuffix = ""
  for opt, arg in opts:
    if opt in ("-h", "--help"):
      usage(sys.argv[0])
      sys.exit()
    elif opt in ('-r', '--host'):
      robotHost = arg 
    elif opt in ('-m', '--mode'):
      mode = arg
    elif opt in ('-c', '--config'):
      loadDefaultValue(argv)
    elif opt in ('-s', '--suffix'):
      refSuffix = arg
  init( robotHost , refSuffix)
  cvp.ref.get_configuration().activate_configuration_set('orange')
  ### YY ###
  init_gui()
  ### YY ###
  
  loop()   

