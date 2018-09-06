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
from functools import partial

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
        
        rospy.init_node('gui_manager')  

        rospy.Subscriber('/base_station/radio_status', RadioStatus, 
            self.radio_cb, queue_size=1)
        rospy.Subscriber('/base_station/camera_status', CameraStatus, 
            self.camera_cb, queue_size=1)
        rospy.Subscriber('/base_station/raw_ctrl', RawCtrl, 
            self.raw_ctrl_cb, queue_size=1)

        self.connect(self, QtCore.SIGNAL("radio_update(PyQt_PyObject)"),
            self.radio_update)
        self.connect(self, QtCore.SIGNAL("camera_update(PyQt_PyObject)"), 
            self.camera_update)
        self.connect(self, QtCore.SIGNAL("raw_ctrl_update(PyQt_PyObject)"), 
            self.raw_ctrl_update)
        
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


    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # launch_simulator(): Launch the simulator ROS launch file.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def launch_simulator(self):

      self.button_simulator.setEnabled(False)
      self.button_rover.setEnabled(False)
      self.button_sandstorm.setEnabled(False)

      run_id = rospy.get_param("/run_id")
      uuid = roslaunch.rlutil.get_or_generate_uuid(run_id, True)
      roslaunch.configure_logging(uuid)

      rospack = rospkg.RosPack() # Get the file path for nova_common
      path = rospack.get_path('nova_common')

      launch_file = [path + '/launch/simulator.launch']

      launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

      launch.start()
      #launch.shutdown()


    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # raw_ctrl_cb(): Xbox controller status callback.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def raw_ctrl_cb(self, msg):
      self.emit(QtCore.SIGNAL("raw_ctrl_update(PyQt_PyObject)"), msg)


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
      
      rospy.wait_for_service('toggle_cam_view')      
      try:          
        client = rospy.ServiceProxy('toggle_cam_view', ToggleStream)          
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
      
      rospy.wait_for_service('toggle_stream')      
      try:          
        client = rospy.ServiceProxy('toggle_stream', ToggleStream)          
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
  
  # 180 deg is North, positive increase is clockwise
  bearing = 135
  ui.dial_bearing.setProperty("value", 180 + bearing)
  ui.label_bearing.setText(str(bearing) + " deg")
  
  voltage = 23.5
  ui.label_voltage.setText(str(voltage) + " V")
  ui.progress_voltage.setProperty("minimum", 230)
  ui.progress_voltage.setProperty("maximum", 252)
  ui.progress_voltage.setProperty("value", round(voltage*10))

  #window.show()
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
    
