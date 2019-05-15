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
import time
import socket
from functools import partial

import rospkg 
import rospy
from gui_ros import GuiRos
from gui_vis import GuiVis

UDP_IP = "192.168.1.8"
UDP_PORT = 6000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

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

    # Setup the GUI in its initial state
    self.setupUi(self)    
    self.GuiRos = GuiRos(self) # Create ROS interface
    self.GuiVis = GuiVis(self) # Create visual element interface
    self.setupConnections()
    self.initialiseGUI()
    self.GuiVis.configureMap()
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
  # initialiseGUI(): 
  #   Setup widget initial values.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def initialiseGUI(self):     

    # Get and set default drive limits
    rpm_limit, steer_limit = self.GuiRos.getDriveLimits("private")    
    self.GuiRos.setDriveLimits(rpm_limit, steer_limit)

    # Initialise GUI widgets
    self.GuiVis.initialiseWidgets(rpm_limit, steer_limit)
                            
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
    self.connect(self, QtCore.SIGNAL("autoUpdate(PyQt_PyObject)"), 
        self.mapUpdate)
    self.GuiVis.setupWidgetConnections()
    self.tool_rosbag_start.toggled.connect(self.GuiRos.toggleRosbag)
    self.tool_900_debug.toggled.connect(self.GuiRos.updateRadioDebug)
    self.tool_5_debug.toggled.connect(self.GuiRos.updateRadioDebug)
             
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

    lat = float(self.edit_lat.text()) # Grab user input lat/lng
    lng = float(self.edit_lng.text())

    self.GuiRos.engageAuto(lat, lng) # Call ROS service to start auto

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # modeChange():  
  #    Function to handle mode changing button presses.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def modeChange(self):
    button = self.sender()
    mode_str = str(button.text())

    # Call ROS service to change mode
    success, _ = self.GuiRos.modeChange(mode_str) 

    # Set button highlight if successful mode change
    if success is True: 
      self.GuiVis.clearModeButtons()
      button.setChecked(True) 
    else:
      button.setChecked(False) 

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # setMission():  
  #    Function to handle mode changing button presses.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
  def setMission(self):
    button = self.sender()
    mission_str = str(button.text())

    # Call ROS service to change mode
    if mission_str == "SCI":
      self.combo_mission.setCurrentIndex(3)
    elif mission_str == "AUT":
      self.combo_mission.setCurrentIndex(2)
    elif mission_str == "ERT" or mission_str == "EQP":
      self.combo_mission.setCurrentIndex(1) 
      
    success, _ = self.GuiRos.setMission(mission_str) 

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # drillCommand():  
  #    Function to handle sending different drill commands
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--   
  def drillCommand(self, value=-1):  
    slider = self.sender()
    name = slider.objectName()
    rospy.loginfo(value) 
    for drill_button in self.GuiVis.drill_buttons:
        drill_button.setChecked(False)
    if value == True:
        slider.setChecked(True)
    if   (name == 'button_ratio_up'): # Ratio up through the pipe
      self.GuiRos.drillCmd(8)
      self.GuiVis.ui.slider_actuator.setValue(1)
      self.GuiVis.ui.slider_drill.setValue(4)
    elif (name == 'button_ratio_down'): # Ratio down through the pipe
      self.GuiRos.drillCmd(7)
      self.GuiVis.ui.slider_actuator.setValue(1)
      self.GuiVis.ui.slider_drill.setValue(4)
    elif (name == 'button_drill_down'):
      self.GuiRos.drillCmd(23)
    elif (name == 'button_retract'):
      self.GuiRos.drillCmd(9)
      time.sleep(0.015)
      self.GuiRos.drillCmd(15)
    elif (name == 'button_drill_stop'):
      self.GuiRos.drillCmd(4)
      self.GuiVis.ui.slider_actuator.setValue(0)
      self.GuiVis.ui.slider_drill.setValue(0)
    elif (name == 'button_actuator_down'):
      self.GuiRos.drillCmd(10)
      time.sleep(0.015)
      self.GuiRos.drillCmd(13+self.GuiVis.ui.slider_actuator.value())

    elif (name == 'button_actuator_up'):
      self.GuiRos.drillCmd(9)
      time.sleep(0.015)
      self.GuiRos.drillCmd(13+self.GuiVis.ui.slider_actuator.value())

    elif (name == 'button_drill_clockwise'):
      self.GuiRos.drillCmd(11)
      time.sleep(0.015)
      self.GuiRos.drillCmd(18+self.GuiVis.ui.slider_drill.value())

    elif (name == 'button_drill_anticlockwise'):
      self.GuiRos.drillCmd(12)
      time.sleep(0.015)
      self.GuiRos.drillCmd(18+self.GuiVis.ui.slider_drill.value())

    elif (name == 'checkBox_sieve'):
      if value==2: #on
         self.GuiRos.drillCmd(24)
      else:
         self.GuiRos.drillCmd(25)
    elif (name == 'checkBox_wet'):
      if value==2: #on
         self.GuiRos.drillCmd(26)
      else:
         self.GuiRos.drillCmd(27)
    elif (name == 'checkBox_pump'):
      if value==2: #on
         self.GuiRos.drillCmd(28)
      else:
         self.GuiRos.drillCmd(29)
    elif (name == 'checkBox_mixers'):
      if value==2: #on
         self.GuiRos.drillCmd(30)
      else:
         self.GuiRos.drillCmd(31)
    elif (name == 'button_science_off'):
       self.GuiRos.drillCmd(25)
       time.sleep(0.015)
       self.GuiRos.drillCmd(27)
       time.sleep(0.015)
       self.GuiRos.drillCmd(29)
       time.sleep(0.015)
       self.GuiRos.drillCmd(31)
       for drill_checkbox in self.GuiVis.drill_checkboxes:
            drill_checkbox.setChecked(False) 
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # initStartup(): 
  #   Disable and enable buttons and panes on startup.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def initStartup(self): 

    self.GuiVis.enableVehicleButtons(False) # Disable vehicle buttons
    self.GuiVis.enableCamPanes(True) # Enable camera rows
    
    self.GuiVis.enableModeButtons(True)  # Enable Mode buttons
    self.button_standby.setChecked(True) # Standby Mode highlight    

    self.stack_mission.removeWidget(self.page_setup) # Remove setup page
    self.combo_mission.removeItem(0)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # launchSimulator(): 
  #   Launch the simulator ROS launch file.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def launchSimulator(self):
  
    self.initStartup() # Disable and enable buttons

    self.label_vehicle.setText("Simulator")
    self.GuiRos.launchSimulator()

    self.combo_mission.setCurrentIndex(1) # Switch to autonomous pane

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # launchJDB(): 
  #   Start up the JDB rover.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def launchJDB(self):
  
    self.initStartup() # Disable and enable buttons

    self.label_vehicle.setText("JDB")
    self.GuiRos.launchJDB()

    self.combo_mission.setCurrentIndex(0) # Switch to autonomous pane

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
    message = "G " + str(msg.latitude) + " " + str(msg.longitude) +  " " + str(msg.bearing)
    self.UDPSend(message)
    
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

    # Get radio widgets
    progress, plugged, paired = self.GuiVis.getRadioWidgets(msg.radio_id)
      
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

    # Get widgets for the particular camera
    led, button_start, button_show = self.GuiVis.getCamWidgets(msg.cam_id)
      
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

    if not success: # Reset button if service call didn't work
      button.setChecked(not checked)
      
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # toggleStream():
  #    Toggle a camera stream on or off.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--         
  def toggleStream(self):
    button = self.sender()
    checked = button.isChecked()
    
    cam_id = int(button.text())
    
    success, _ = self.GuiRos.toggleStream(cam_id, checked)

    if not success: # Reset button if service call didn't work
      button.setChecked(not checked)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # toggleRadioDebug():    
  #    Toggle whether or not we are in radio debug mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def toggleRadioDebug(self):
    button = self.sender()
    checked = button.isChecked()

    self.GuiRos.toggleRadioDebug(checked)
    self.GuiVis.enableRadioDebug(checked)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # moveGimbal():    
  #    Respond to a gimbal command button press.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def moveGimbal(self):
    button = self.sender()

    cmd = self.GuiVis.getGimbalCmdFromButton(button.objectName())
    self.GuiRos.controlGimbal(1, cmd) # TODO set gimbal cam_id as param

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # stopGimbal():    
  #    Respond to a gimbal command button release
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def stopGimbal(self):
    self.GuiRos.controlGimbal(1, 'ptzStopRun') # TODO set gimbal cam_id as param

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # toggleGimbalZoom():    
  #    Stop the gimbal from moin
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def toggleGimbalZoom(self):
    button = self.sender()
    checked = button.isChecked()

    if checked is True:
      cmd = 'zoomIn'
    else:
      cmd = 'zoomOut'
    
    self.GuiRos.controlGimbal(1, cmd) # TODO set gimbal cam_id as param

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # mapUpdate()    
  #    update map
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def mapUpdate(self,msg):
    self.GuiVis.data[0] = (msg.latitude,msg.longitude)
    self.GuiVis.update()
   
  def plot(self):
      x = self.ui.latitudeSpinBox.value()
      y = self.ui.longitudeSpinBox.value()
      self.GuiVis.data.append((x,y))
      
      self.update()


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # UDPSend()    
  #    send data to other base station computer via UDP protocol
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def UDPSend(self,msg):
    rospy.loginfo(msg)
    sock.sendto(msg, (UDP_IP, UDP_PORT))
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
    
