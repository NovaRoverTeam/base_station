#!/usr/bin/env python

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# File: gui_ros.py
# Author: Ben Steer
# Last modified by: Ben Steer
#
# Description:
#   The gui_ros node acts as the GUI's interface to ROS. The 
#   gui_backend.py script contains purely Qt-related Python and no
#   ROS activities (except for accessing values from ROS msgs), whereas
#   gui_ros.py contains purely ROS-related code. In this way, gui_ros 
#   encapsulates all interaction with the ROS network from the 
#   perspective of the GUI.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

import rospy
import rospkg
import roslaunch
from base_station.msg import *
from nova_common.msg import * 
from nova_common.srv import *

from PyQt4 import QtCore

# Class representing the ROS interface
class GuiRos():

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # __init__():    #
    #    Initiases main class, ROS node and sets up subs/pubs/srvs.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
    def __init__(self, ui):

        self.srv_timeout = 5   # Number of seconds before service timeout
        self.bag_name = 'test' # Rosbag default filename
        self.ui = ui  # Qt MainDialog (for connecting ROS msg callbacks)
        self.min_signal = -96 # Minimum signal strength, dB
      
        rospy.init_node('gui_ros')

        rospy.Subscriber('/base_station/radio_status', RadioStatus, 
          self.radioCb, queue_size=1)
        rospy.Subscriber('/core_rover/camera_status', CameraStatus, 
          self.cameraCb, queue_size=1)
        rospy.Subscriber('/core_rover/auto_status', AutoStatus, 
          self.autoCb, queue_size=1)
        rospy.Subscriber('/base_station/raw_ctrl', RawCtrl, 
          self.rawCtrlCb, queue_size=1)

        self.gimbal_pub = rospy.Publisher('/base_station/gimbal_cmd', 
            GimbalCmd, queue_size=10)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # getMode():   
    #   Get the current base station Mode.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def getMode(self): 
        return rospy.get_param('/base_station/Mode')

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # toggleRosbag():   
    #   Toggle recording of a rosbag session.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def toggleRosbag(self, toggled):  
    #global recording 
    
        if toggled:  
            print("gon rosbag now")
            #date_str = datetime.date.today().strftime("%Y_%B_%d_%I:%M")
            #name_str = self.bag_name + '_' + date_str
            #bash_cmd = 'rosbag record -a -O ' + name_str
            #rosbag_recorder = pexpect.spawn(bash_cmd)    
    
        else:
            print("dun rosbaggin")
            #rosbag_recorder.sendintr()
            #rosbag_recorder.close() 

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # shutdownRos():   
    #   Shut down the ROS system and any launched systems.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def shutdownRos(self):  

        self.launch.shutdown() # Shut down roslaunches
        rospy.signal_shutdown("SIGINT") # Shut down node

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # getDriveLimits():   
    #   Retrieve values of speed and turning limits. "source" indicates
    #   whether to retrieve from private or global param server.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def getDriveLimits(self, source):  

        if source is "private": # Get default limits
            rpm_limit   = rospy.get_param('~def_rpm_limit') 
            steer_limit = rospy.get_param('~def_steer_limit')
        else:
            rpm_limit   = rospy.get_param('rpm_limit') 
            steer_limit = rospy.get_param('steer_limit')

        return rpm_limit, steer_limit

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # setDriveLimits():   
    #   Set values of speed and turning limits. If parameter is -1, don't
    #   change it.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def setDriveLimits(self, rpm_limit, steer_limit):  

        if rpm_limit >= 0:
            rospy.set_param('rpm_limit', rpm_limit) 
        if steer_limit >= 0:
            rospy.set_param('steer_limit', steer_limit)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # engageAuto():
    #    Function to begin an autonomous mission.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def engageAuto(self, lat, lng):

      try:                  
        rospy.wait_for_service('/core_rover/start_auto', self.srv_timeout)
        client = rospy.ServiceProxy('/core_rover/start_auto', StartAuto)

        req = StartAutoRequest()
        req.latitude  = lat
        req.longitude = lng

        res = client(req)
        rospy.loginfo(res.message)
                  
      except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s"%e)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # setMission(): 
    #    Handles changing the Mission parameter.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def setMission(self, mission_str):

        # Request change of mode from rover
        try:    
            rospy.wait_for_service('/base_station/change_mission', self.srv_timeout)      
            client = rospy.ServiceProxy('/base_station/change_mission',
              ChangeMode)
            res = client(mission_str)
        
            # Return success or failure of mode change
            rospy.loginfo(res.message)
            return res.success, res.message
                  
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)
            return False, "%s"%e

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # modeChange(): 
    #    Function to handle mode changing button presses.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def modeChange(self, mode_str):

        # Request change of mode from rover
        try:    
            rospy.wait_for_service('/base_station/change_mode', self.srv_timeout)      
            client = rospy.ServiceProxy('/base_station/change_mode',
              ChangeMode)
            res = client(mode_str)
        
            # Return success or failure of mode change
            rospy.loginfo(res.message)
            return res.success, res.message
                  
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)
            return False, "%s"%e

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # launchSimulator(): 
    #   Launch the simulator ROS launch file.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def launchSimulator(self):

        run_id = rospy.get_param("/run_id")
        uuid = roslaunch.rlutil.get_or_generate_uuid(run_id, True)
        roslaunch.configure_logging(uuid)

        rospack = rospkg.RosPack() # Get the file path for nova_common
        path = rospack.get_path('nova_common')

        launch_file = [path + '/launch/simulator.launch']

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
        self.launch.start() # Start the launch file
      
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # rawCtrlCb(): 
    #   Xbox controller status callback.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def rawCtrlCb(self, msg):
        self.ui.emit(QtCore.SIGNAL("rawCtrlUpdate(PyQt_PyObject)"), msg)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # autoCb(): 
    #   Auto status callback.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def autoCb(self, msg):
        self.ui.emit(QtCore.SIGNAL("autoUpdate(PyQt_PyObject)"), msg)
  
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # radioCb(): 
    #   Radio status callback.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def radioCb(self, msg):
        self.ui.emit(QtCore.SIGNAL("radioUpdate(PyQt_PyObject)"), msg)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # cameraCb(): 
    #   Camera status callback.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--       
    def cameraCb(self, msg):
        self.ui.emit(QtCore.SIGNAL("cameraUpdate(PyQt_PyObject)"), msg)

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # toggleCamView():
    #    Bring up a video window for a camera stream.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--      
    def toggleCamView(self, cam_id, checked):
        
        try:          
            rospy.wait_for_service('/base_station/toggle_cam_view', self.srv_timeout)
            client = rospy.ServiceProxy('/base_station/toggle_cam_view', ToggleStream)          
            res = client(cam_id, checked)
        
            return res.success, res.message
        
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)
            return False, ""

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # toggleStream():    
    #    Toggle a camera stream on or off.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def toggleStream(self, cam_id, checked):

        try:          
            rospy.wait_for_service('/base_station/toggle_stream', self.srv_timeout)
            client = rospy.ServiceProxy('/base_station/toggle_stream', ToggleStream)          
            res = client(cam_id, checked)      

            return res.success, res.message       
          
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)
            return False, ""

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # toggleRadioDebug():    
    #    Toggle whether or not we are in radio debug mode.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def toggleRadioDebug(self, checked):

        rospy.set_param('/RadioDebug', checked)

        if checked:
            rospy.loginfo("Radio debugging enabled.")
        else:
            rospy.loginfo("Radio debugging disabled.")

        self.updateRadioDebug()

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # updateRadioDebug():    
    #    Refresh data from radio debug buttons.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def updateRadioDebug(self):

        checked = self.ui.tool_900_debug.isChecked()
        rospy.set_param('/RadioDebug9', checked)
        self.ui.radioUpdate(RadioStatus(0, checked, self.min_signal, checked))

        checked = self.ui.tool_5_debug.isChecked()
        rospy.set_param('/RadioDebug5', checked)
        self.ui.radioUpdate(RadioStatus(1, checked, self.min_signal, checked))

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # controlGimbal():    
    #    Send a gimbal control commmand message.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def controlGimbal(self, cam_id, cmd):
        self.gimbal_pub.publish(GimbalCmd(cam_id, cmd))