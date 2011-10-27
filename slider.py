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


# Slider Panel for 0 to 256
class SliderPanel( JPanel, ChangeListener ):

  def __init__( self, rtc, conf_set_name, param_name, inival=128, minval=0, maxval=256 ):
    bdcolor = Color( 32, 32, 32, 32 )
    lborder = LineBorder( Color( 32, 32, 32, 32 ), 1, False )
    
    self.rtc = rtc
    self.conf_set_name = conf_set_name
    self.param_name = param_name
    
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
    
    self.valtext.setText( str( self.getSliderValue() )[:4] )
  
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
      slider = SliderPanel( rtc, cs.id, key, int( conf_set_dict[cs.id][key] ) )
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
      if key == 'brightness' :
      #  print '%s : %s = %s' % ( cs.id, key, conf_set_dict[cs.id][key] )
        slider = SliderPanel_Percent( rtc, cs.id, key, float( conf_set_dict[cs.id][key] ) )
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

  rtm.connectPorts(vs.port("MultiCameraImages"), cvp.port("MultiCameraImage"))


def loop():
  vs_svc.take_one_frame()
  time.sleep(2)
  while 1:
    vs_svc.take_one_frame()
#    time.sleep(0.5)
    time.sleep(2) # ThinkPad X200


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


if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None
  init( robotHost )
  cvp.ref.get_configuration().activate_configuration_set('orange')
  
  ### YY ###
  init_gui()
  ### YY ###
  
  loop()   

