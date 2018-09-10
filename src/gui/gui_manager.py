#!/usr/bin/env python

import rospy
import roslaunch
import rospkg 
from base_station.msg import *
from nova_common.msg import * 
from nova_common.srv import *

import sys
import datetime
import pexpect
import signal
import subprocess
from functools import partial

# Get the file path for base_station, rebuild GUI
rospack = rospkg.RosPack() 
base_path = rospack.get_path('base_station')
bash_cmd = "cd " + base_path + "/src/gui;" + " ./build.sh"
output = subprocess.check_output(['bash','-c', bash_cmd])  

from PyQt4 import QtCore, QtGui # Qt includes
from PyKDE4.kdeui import *
from main import Ui_MainWindow

bag_name = 'test'


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# rosbag_start():
#
#    Start recording a rosbag session.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def rosbag_start(toggled):  
  #global recording 
  
  if toggled:  
    print("gon rosbag now")
    #date_str = datetime.date.today().strftime("%Y_%B_%d_%I:%M")
    #name_str = bag_name + '_' + date_str
    #bash_cmd = 'rosbag record -a -O ' + name_str
    #rosbag_recorder = pexpect.spawn(bash_cmd)    
  
  else:
    print("dun rosbaggin")
    #rosbag_recorder.sendintr()
    #rosbag_recorder.close() 
        
    
class MainDialog(QtGui.QMainWindow, Ui_MainWindow):

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # __init__():
    #
    #    Initiases main class, ROS node and sets up subs/pubs/srvs.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
    def __init__(self, parent=None):
      super(MainDialog,self).__init__(parent)
      self.setupUi(self)

      self.srv_timeout = 3 # Number of seconds to wait for service
      
      rospy.init_node('gui_manager')  

      signal.signal(signal.SIGINT, self.signal_handler) # Register sigint handler

      rospy.Subscriber('/base_station/radio_status', RadioStatus, 
          self.radio_cb, queue_size=1)
      rospy.Subscriber('/core_rover/camera_status', CameraStatus, 
          self.camera_cb, queue_size=1)
      rospy.Subscriber('/core_rover/auto_status', AutoStatus, 
          self.auto_cb, queue_size=1)
      rospy.Subscriber('/base_station/raw_ctrl', RawCtrl, 
          self.raw_ctrl_cb, queue_size=1)
          
      self.make_connections()
      self.setup_widgets()
                 
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # signal_handler(): Override default exit handler for CTRL+C SIGINT.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
    def signal_handler(self, sig, frame):
      self.launch.shutdown()
      rospy.signal_shutdown("SIGINT") # Shut down ROS   
      sys.exit(1)


    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # closeEvent(): Override default exit handler for clean GUI shutdown.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
    def closeEvent(self, event):
      self.launch.shutdown()
      rospy.signal_shutdown("SIGINT") # Shut down node
      event.accept()
      
     
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # setup_widgets(): Setup widget initial values and ROS params.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def setup_widgets(self): 
      self.combo_mission.setCurrentIndex(0) # Switch to setup pane
      self.stack_mission.setCurrentIndex(0) 
          
      rpm_limit   = rospy.get_param('~def_RPM_limit') # Get default limits
      steer_limit = rospy.get_param('~def_steer_limit')
    
      rospy.set_param('RPM_limit', rpm_limit) # Set ROS parameters
      rospy.set_param('steer_limit', steer_limit)
      
      self.slider_a.setProperty("value", round(rpm_limit*100))
      self.slider_b.setProperty("value", round(steer_limit*100))  
      
      self.edit_lat.setValidator(QtGui.QDoubleValidator() )                       
      self.edit_lng.setValidator(QtGui.QDoubleValidator() )    
                         
            
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # make_connections(): Setup widget connections for signals.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def make_connections(self): 
      self.connect(self, QtCore.SIGNAL("radio_update(PyQt_PyObject)"),
          self.radio_update)
      self.connect(self, QtCore.SIGNAL("camera_update(PyQt_PyObject)"), 
          self.camera_update)
      self.connect(self, QtCore.SIGNAL("raw_ctrl_update(PyQt_PyObject)"), 
          self.raw_ctrl_update)
      self.connect(self, QtCore.SIGNAL("auto_update(PyQt_PyObject)"), 
          self.auto_update)
            
      self.tool_cam0_show.clicked.connect(self.toggle_cam_view)
      self.tool_cam1_show.clicked.connect(self.toggle_cam_view)
      self.tool_cam2_show.clicked.connect(self.toggle_cam_view)
      self.tool_cam3_show.clicked.connect(self.toggle_cam_view)
      self.tool_cam4_show.clicked.connect(self.toggle_cam_view)
      
      self.tool_cam0_start.clicked.connect(self.toggle_stream)
      self.tool_cam1_start.clicked.connect(self.toggle_stream)
      self.tool_cam2_start.clicked.connect(self.toggle_stream)
      self.tool_cam3_start.clicked.connect(self.toggle_stream)
      self.tool_cam4_start.clicked.connect(self.toggle_stream)

      self.button_simulator.clicked.connect(self.launch_simulator)
      
      self.button_standby.clicked.connect(self.mode_change)
      self.button_drive  .clicked.connect(self.mode_change)
      self.button_arm    .clicked.connect(self.mode_change)
      self.button_drill  .clicked.connect(self.mode_change)
      self.button_auto   .clicked.connect(self.mode_change)     
      
      self.slider_a.valueChanged.connect(self.slider_change)
      self.slider_b.valueChanged.connect(self.slider_change)  
      self.slider_c.valueChanged.connect(self.slider_change)  
      
      self.tool_slider_a.toggled.connect(self.slider_unlock)
      self.tool_slider_b.toggled.connect(self.slider_unlock)  
      self.tool_slider_c.toggled.connect(self.slider_unlock)  
        
        
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # slider_change():
    #
    #    Function to handle slider values being changed.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def slider_change(self, value):  
      slider = self.sender()
      name = slider.objectName()
      
      if   (name == 'slider_a'): # Setting RPM limit
        rospy.set_param('RPM_limit',   float(value)/100)
        
      elif (name == 'slider_b'): # Setting steering limit
        rospy.set_param('steer_limit', float(value)/100)
   
   
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # slider_unlock():
    #
    #    Lock or unlock a slider using the tool buttons.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def slider_unlock(self, unlock):  
      slider = self.sender()
      name = slider.objectName()
      
      if   (name == 'tool_slider_a'): 
        self.slider_a.setEnabled(unlock)
        
      elif (name == 'tool_slider_b'):
        self.slider_b.setEnabled(unlock)
        
      else:
        self.slider_c.setEnabled(unlock)
           
      
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # mode_change():
    #
    #    Function to handle mode changing button presses.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def mode_change(self):
      button = self.sender()
      mode_str = str(button.text())
      
      # Remove highlight from previous mode's button
      # TODO can be a bit buggy, not quite sure what default does exactly
      self.button_standby.setProperty('default', False)
      self.button_drive.setProperty('default', False)
      self.button_arm.setProperty('default', False)
      self.button_drill.setProperty('default', False)
      self.button_auto.setProperty('default', False)
      
      # Request change of mode from rover
      rospy.wait_for_service('/base_station/change_state', self.srv_timeout)
      try:          
        client = rospy.ServiceProxy('/base_station/change_state',
          ChangeState)
        res = client('BaseMode', mode_str)
        
        # If successfully changed mode, highlight new button
        if res.success:
          button.setProperty('default', True)
                  
      except rospy.ServiceException, e:
        rospy.loginfo("Changing mode failed: %s"%e)
      
      
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # init_setup_buttons(): Disable and enable buttons on startup.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def init_setup_buttons(self): 
      self.button_simulator.setEnabled(False) # Disable setup buttons
      self.button_rover    .setEnabled(False)
      self.button_sandstorm.setEnabled(False)   
      
      self.button_standby.setEnabled(True) # Enable mode buttons
      self.button_drive  .setEnabled(True)
      self.button_arm    .setEnabled(True)
      self.button_drill  .setEnabled(True)
      self.button_auto   .setEnabled(True)
      
      self.button_standby.setProperty('default', True) # Standby highlight


    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # launch_simulator(): Launch the simulator ROS launch file.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def launch_simulator(self):
    
      self.init_setup_buttons() # Disable and enable buttons
      self.combo_mission.setCurrentIndex(2) # Switch to autonomous pane
      
      self.stack_mission.removeWidget(self.page_setup) # Remove setup page
      self.combo_mission.removeItem(0)
      
      self.label_vehicle.setText("Simulator")
      
      run_id = rospy.get_param("/run_id")
      uuid = roslaunch.rlutil.get_or_generate_uuid(run_id, True)
      roslaunch.configure_logging(uuid)

      rospack = rospkg.RosPack() # Get the file path for nova_common
      path = rospack.get_path('nova_common')

      launch_file = [path + '/launch/simulator.launch']

      self.launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

      self.launch.start()


    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # raw_ctrl_cb(): Xbox controller status callback.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def raw_ctrl_cb(self, msg):
      self.emit(QtCore.SIGNAL("raw_ctrl_update(PyQt_PyObject)"), msg)


    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # auto_cb(): Radio status callback.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def auto_cb(self, msg):
      self.emit(QtCore.SIGNAL("auto_update(PyQt_PyObject)"), msg)
      
      
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # radio_cb(): Radio status callback.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def radio_cb(self, msg):
      self.emit(QtCore.SIGNAL("radio_update(PyQt_PyObject)"), msg)


    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # camera_cb(): Camera status callback.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--       
    def camera_cb(self, msg):
      self.emit(QtCore.SIGNAL("camera_update(PyQt_PyObject)"), msg)
 
 
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # auto_update():
    #
    #    Update the rover with autonomous mode data including GPS, IMU.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
    def auto_update(self, msg):      
      
      self.label_auto_state.setText(str(msg.auto_state))
      
      self.label_lat.setText(str(msg.latitude))
      self.label_lng.setText(str(msg.longitude))
      
      # 180 deg is North, positive increase is clockwise
      bearing = round(msg.bearing)
      self.dial_bearing.setProperty("value", 180 + bearing)
      self.label_bearing.setText(str(bearing))
      

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # raw_ctrl_update():
    #
    #    Update the Input Status pane of the GUI.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
    def raw_ctrl_update(self, msg):   

      led = self.led_xbox

      if msg.connected is True:
        led.on()
      else:
        led.off()
  
  
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # radio_update():
    #
    #    Update the Radio Status pane of the GUI.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
    def radio_update(self, msg):       
      if msg.radio_id is 0:                # Radio 0, 900 MHz
        progress = self.progress_radio0
        plugged  = self.led_radio0_plugged
        paired   = self.led_radio0_paired
      else:                                # Radio 1, 5.8 GHz
        progress = self.progress_radio1
        plugged  = self.led_radio1_plugged
        paired   = self.led_radio1_paired
        
      progress.setProperty("value", msg.signal)
      
      if msg.ssh_active:
        plugged.on()
      else:
        plugged.off()        
        
      if msg.n_wlan_cons > 0:
        paired.on()
      else:
        paired.off()
        
        
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # camera_update():
    #
    #    Update the Radio Status pane of the GUI.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
    def camera_update(self, msg):       
      if msg.cam_id is 0:
        led = self.led_cam0
        button_start  = self.tool_cam0_start
        button_show   = self.tool_cam0_show
      elif msg.cam_id is 1:
        led = self.led_cam1
        button_start  = self.tool_cam1_start
        button_show   = self.tool_cam1_show
      elif msg.cam_id is 2:
        led = self.led_cam2
        button_start  = self.tool_cam2_start
        button_show   = self.tool_cam2_show
      elif msg.cam_id is 3:
        led = self.led_cam3
        button_start  = self.tool_cam3_start
        button_show   = self.tool_cam3_show
      elif msg.cam_id is 4:
        led = self.led_cam4
        button_start  = self.tool_cam4_start
        button_show   = self.tool_cam4_show
        
      button_start.setChecked(msg.streaming)
      
      # If show video button is on while streaming stopped, uncheck it
      if button_show.isChecked() and not msg.streaming:
        button_show.setChecked(False)
      
      if msg.streaming is True:
        led.on()
      else:
        led.off()
        
        
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # toggle_cam_view():
    #
    #    Bring up a video window for a camera stream.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def toggle_cam_view(self):
      button = self.sender()
      checked = button.isChecked()
      
      cam_id = int(button.text())
      
      rospy.wait_for_service('/base_station/toggle_cam_view', self.srv_timeout)      
      try:          
        client = rospy.ServiceProxy('/base_station/toggle_cam_view', ToggleStream)          
        res = client(cam_id, checked)
        
        if not res.success: # If stream inactive, release button      
          button.setChecked(False)
          
      except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s"%e)
        
    
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # toggle_stream():
    #
    #    Toggle a camera stream on or off
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--         
    def toggle_stream(self):
      button = self.sender()
      checked = button.isChecked()
      
      cam_id = int(button.text())
      
      rospy.wait_for_service('/base_station/toggle_stream', self.srv_timeout)      
      try:          
        client = rospy.ServiceProxy('/base_station/toggle_stream', ToggleStream)          
        res = client(cam_id, checked)             
          
      except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s"%e)
          
   
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def main():
      
  app = QtGui.QApplication(sys.argv)
  
  ui = MainDialog()    
  ui.tool_rosbag_start.toggled.connect(rosbag_start)
    
  voltage = 23.0
  ui.label_voltage.setText(str(voltage) + " V")
  ui.progress_voltage.setProperty("minimum", 230)
  ui.progress_voltage.setProperty("maximum", 252)
  ui.progress_voltage.setProperty("value", round(voltage*10))

  ui.show()   
  app.exec_()
  
  
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
    
