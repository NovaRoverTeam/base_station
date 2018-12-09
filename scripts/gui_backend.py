#!/usr/bin/env python

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# File: gui_backend.py
# Author: Ben Steer
# Last modified by: Ben Steer
#
# Description:
#   The gui_backend.py script contains purely Qt-related Python and no
#   ROS activities, making use of the gui_ros module to interface with
#   the ROS network. This script is not a ROS node, but instantiates
#   a ROS node through gui_ros. The purpose of gui_backend is to 
#   configure the visual elements of the GUI, connect buttons and
#   widgets to their associated functions, display data and take user
#   input from fields.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

import sys
import datetime
import pexpect
import signal
import subprocess
from functools import partial

import rospkg 
import rospy
from gui_ros import GuiRos

# Get the file path for base_station, rebuild GUI
rospack = rospkg.RosPack() 
base_path = rospack.get_path('base_station')
bash_cmd = "cd " + base_path + "/scripts/gui;" + " ./build.sh"
output = subprocess.check_output(['bash','-c', bash_cmd])  

from PyQt4 import QtCore, QtGui # Qt includes
from PyKDE4.kdeui import *
from gui.main import Ui_MainWindow

# Class representing the main dialog window of the GUI
class MainDialog(QtGui.QMainWindow, Ui_MainWindow):

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # __init__():
  #    Initiases main class, starts gui_ros and perform initial setup.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def __init__(self, parent=None):
    super(MainDialog,self).__init__(parent)

    self.GuiRos = GuiRos(self) # Create ROS interface

    self.setupUi(self)          
    self.setupConnections()
    self.setupWidgets()

    # Register sigint handler
    signal.signal(signal.SIGINT, self.signalHandler) 
                
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # signalHandler(): 
  #   Override default exit handler for CTRL+C SIGINT.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def signalHandler(self, sig, frame):
    self.GuiRos.shutdownRos()   
    sys.exit(1)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # closeEvent(): 
  #   Override default exit handler for clean GUI shutdown.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def closeEvent(self, event):
    self.GuiRos.shutdownRos()
    event.accept()
    
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # setupWidgets(): 
  #   Setup widget initial values and ROS params.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def setupWidgets(self): 
    self.combo_mission.setCurrentIndex(0) # Switch to setup pane
    self.stack_mission.setCurrentIndex(0) 
        
    # Get and set default drive limits
    rpm_limit, steer_limit = self.GuiRos.getDriveLimits("private")    
    self.GuiRos.setDriveLimits(rpm_limit, steer_limit)
    
    self.slider_a.setProperty("value", round(rpm_limit*100))
    self.slider_b.setProperty("value", round(steer_limit*100))  
    
    self.edit_lat.setValidator(QtGui.QDoubleValidator() )                       
    self.edit_lng.setValidator(QtGui.QDoubleValidator() )    

    mode_stylesheet = "QPushButton:checked { background-color: orange; }\n"
    self.button_standby.setStyleSheet(mode_stylesheet)
    self.button_drive.setStyleSheet(mode_stylesheet)
    self.button_arm.setStyleSheet(mode_stylesheet)
    self.button_drill.setStyleSheet(mode_stylesheet)
    self.button_auto.setStyleSheet(mode_stylesheet)

    leds = [self.led_GPS, self.led_LIDAR, self.led_Raman, self.led_haptic, 
      self.led_xbox, self.led_IMU, self.led_rosbag, self.led_battery, 
      self.led_relay, self.led_cam0, self.led_cam1, self.led_cam2, 
      self.led_cam3, self.led_cam4]

    for led in leds:
      led.off()

    self.cam0_pane.setEnabled(False)
    self.cam1_pane.setEnabled(False)
    self.cam2_pane.setEnabled(False)
    self.cam3_pane.setEnabled(False)
    self.cam4_pane.setEnabled(False)
                            
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # setupConnections(): 
  #   Setup widget connections for signals.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def setupConnections(self): 
    
    self.connect(self, QtCore.SIGNAL("radioUpdate(PyQt_PyObject)"),
        self.radioUpdate)
    self.connect(self, QtCore.SIGNAL("cameraUpdate(PyQt_PyObject)"), 
        self.cameraUpdate)
    self.connect(self, QtCore.SIGNAL("rawCtrlUpdate(PyQt_PyObject)"), 
        self.rawCtrlUpdate)
    self.connect(self, QtCore.SIGNAL("autoUpdate(PyQt_PyObject)"), 
        self.autoUpdate)
          
    self.tool_cam0_show.clicked.connect(self.toggleCamView)
    self.tool_cam1_show.clicked.connect(self.toggleCamView)
    self.tool_cam2_show.clicked.connect(self.toggleCamView)
    self.tool_cam3_show.clicked.connect(self.toggleCamView)
    self.tool_cam4_show.clicked.connect(self.toggleCamView)
    
    self.tool_cam0_start.clicked.connect(self.toggleStream)
    self.tool_cam1_start.clicked.connect(self.toggleStream)
    self.tool_cam2_start.clicked.connect(self.toggleStream)
    self.tool_cam3_start.clicked.connect(self.toggleStream)
    self.tool_cam4_start.clicked.connect(self.toggleStream)

    self.button_simulator.clicked.connect(self.launchSimulator)
    self.button_engage_auto.clicked.connect(self.engageAuto)
    
    self.button_standby.clicked.connect(self.modeChange)
    self.button_drive  .clicked.connect(self.modeChange)
    self.button_arm    .clicked.connect(self.modeChange)
    self.button_drill  .clicked.connect(self.modeChange)
    self.button_auto   .clicked.connect(self.modeChange)     
    
    self.slider_a.valueChanged.connect(self.sliderChange)
    self.slider_b.valueChanged.connect(self.sliderChange)  
    self.slider_c.valueChanged.connect(self.sliderChange)  
    
    self.tool_slider_a.toggled.connect(self.sliderUnlock)
    self.tool_slider_b.toggled.connect(self.sliderUnlock)  
    self.tool_slider_c.toggled.connect(self.sliderUnlock)  

    self.tool_rosbag_start.toggled.connect(self.GuiRos.toggleRosbag)
         
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # sliderChange():
  #    Function to handle slider values being changed.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def sliderChange(self, value):  
    slider = self.sender()
    name = slider.objectName()
    
    if   (name == 'slider_a'): # Set RPM limit
      self.GuiRos.setDriveLimits(float(value)/100, -1)
      
    elif (name == 'slider_b'): # Set steering limit
      self.GuiRos.setDriveLimits(-1, float(value)/100)
  
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # sliderUnlock():
  #    Lock or unlock a slider using the tool buttons.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def sliderUnlock(self, unlock):  
    slider = self.sender()
    name = slider.objectName()
    
    if   (name == 'tool_slider_a'): 
      self.slider_a.setEnabled(unlock)
      
    elif (name == 'tool_slider_b'):
      self.slider_b.setEnabled(unlock)
      
    else:
      self.slider_c.setEnabled(unlock)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # engageAuto():    #
  #    Function to begin an autonomous mission.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def engageAuto(self):

    lat = float(self.edit_lat.text())
    lng = float(self.edit_lng.text())

    self.GuiRos.engageAuto(lat, lng) # Call ROS service to start auto

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # modeChange():  
  #    Function to handle mode changing button presses.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def modeChange(self):
    button = self.sender()
    mode_str = str(button.text())
    
    # Remove highlight from previous mode's button
    # TODO can be a bit buggy, not quite sure what default does exactly

    # Call service to change mode
    success, _ = self.GuiRos.modeChange(mode_str) 

    if success is True: # Set button highlight
      self.button_standby.setChecked(False)
      self.button_drive.setChecked(False)
      self.button_arm.setChecked(False)
      self.button_drill.setChecked(False)
      self.button_auto.setChecked(False)
      button.setChecked(True) 
    else:
      button.setChecked(False) 

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # initSetupButtons(): 
  #   Disable and enable buttons on startup.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def initSetupButtons(self): 
    self.button_simulator.setEnabled(False) # Disable setup buttons
    self.button_rover    .setEnabled(False)
    self.button_sandstorm.setEnabled(False)   
    
    self.button_standby.setEnabled(True) # Enable mode buttons
    self.button_drive  .setEnabled(True)
    self.button_arm    .setEnabled(True)
    self.button_drill  .setEnabled(True)
    self.button_auto   .setEnabled(True)
    
    self.button_standby.setChecked(True) # Standby highlight

    self.cam0_pane.setEnabled(True)
    self.cam1_pane.setEnabled(True)
    self.cam2_pane.setEnabled(True)
    self.cam3_pane.setEnabled(True)
    self.cam4_pane.setEnabled(True)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # launchSimulator(): 
  #   Launch the simulator ROS launch file.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def launchSimulator(self):
  
    self.initSetupButtons() # Disable and enable buttons
    self.combo_mission.setCurrentIndex(2) # Switch to autonomous pane
    
    self.stack_mission.removeWidget(self.page_setup) # Remove setup page
    self.combo_mission.removeItem(0)
    
    self.label_vehicle.setText("Simulator")

    self.GuiRos.launchSimulator()

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # autoUpdate():
  #    Update the rover with autonomous mode data including GPS, IMU.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def autoUpdate(self, msg):      
    
    self.label_auto_state.setText(str(msg.auto_state))
    
    self.label_lat.setText(str(msg.latitude))
    self.label_lng.setText(str(msg.longitude))
    
    # 180 deg is North, positive increase is clockwise
    bearing = round(msg.bearing)
    self.dial_bearing.setProperty("value", 180 + bearing)
    self.label_bearing.setText(str(bearing))
    
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # rawCtrlUpdate():
  #    Update the Input Status pane of the GUI.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def rawCtrlUpdate(self, msg):   

    led = self.led_xbox

    if msg.connected is True:
      led.on()
    else:
      led.off()

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # radioUpdate():
  #    Update the Radio Status pane of the GUI.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def radioUpdate(self, msg):      

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
  # cameraUpdate():
  #    Update the Camera pane of the GUI.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def cameraUpdate(self, msg): 

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
      self.GuiRos.toggleCamView(msg.cam_id, False) # TODO test this
      button_show.setChecked(False)
    
    if msg.streaming is True:
      led.on()
    else:
      led.off()
       
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # toggleCamView():
  #    Bring up a video window for a camera stream.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def toggleCamView(self):
    button = self.sender()
    checked = button.isChecked()
    
    cam_id = int(button.text())
    
    success, _ = self.GuiRos.toggleCamView(cam_id, checked)
    button.setChecked(success)
      
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # toggleStream():
  #    Toggle a camera stream on or off.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--         
  def toggleStream(self):
    button = self.sender()
    checked = button.isChecked()
    
    cam_id = int(button.text())
    
    success, _ = self.GuiRos.toggleStream(cam_id, checked)
    button.setChecked(success)

   
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def main():
      
  app = QtGui.QApplication(sys.argv)
  
  ui = MainDialog()    
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
    