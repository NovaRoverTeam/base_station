#!/usr/bin/env python

import rospy
from nova_common.msg import *
from nova_common.srv import *
from std_msgs.msg import Empty
from std_msgs.msg import Int8

speed_change_flag = False
class BaseSync:

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # __init__():
  #    Initialise class.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def __init__(self):

    rospy.init_node('base_sync') 

    self.initialiseState() # Set initial parameter server values

    self.srv_timeout = 5   # Number of seconds before service timeout

    self.raw_ctrl_sub   = rospy.Subscriber(
      '/base_station/xbox_raw_ctrl', RawCtrl, self.rawCtrlCb)

    self.right_ctrl_sub = rospy.Subscriber('/base_station/rjs_raw_ctrl',RawCtrl, self.rightRawCtrlCb) 

    self.change_mode_server = rospy.Service(
      '/base_station/change_mode', ChangeMode, 
      self.handleChangeMode)

    self.change_mission_server = rospy.Service(
      '/base_station/change_mission', ChangeMission, 
      self.handleChangeMission)
        
    self.drive_cmd_pub = rospy.Publisher(
      '/core_rover/driver/drive_cmd', DriveCmd, 
        queue_size=1)

    self.max_speed_pub = rospy.Publisher(
      '/base_station/change_max_speed', Int8, 
        queue_size=1)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # handleChangeMode():
  #  Service server handler for changing rover mode. Calls service in
  #  rover_sync to confirm mode change.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def handleChangeMode(self, req):    
    try:
      rospy.wait_for_service('/core_rover/req_change_mode', self.srv_timeout)
      client = rospy.ServiceProxy('/core_rover/req_change_mode',
        ChangeMode)
      res_rover = client(req.mode)
        
    except rospy.ServiceException, e:
      rospy.loginfo("Changing mode failed: %s"%e)
      return ChangeModeResponse(False, "%s"%e)
    
    if res_rover.success:
      rospy.set_param('/base_station/Mode', req.mode)
      res_base = ChangeModeResponse(True, res_rover.message)
    else:
      res_base = ChangeModeResponse(False, res_rover.message)
              
    return res_base

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # handleChangeMission():
  #  Service server handler for changing rover mission.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def handleChangeMission(self, req):    
    try:
      rospy.wait_for_service('/core_rover/req_change_mission', self.srv_timeout)
      client = rospy.ServiceProxy('/core_rover/req_change_mission',
        ChangeMission)
      res_rover = client(req.mission)
        
    except rospy.ServiceException, e:
      rospy.loginfo("Changing mission failed: %s"%e)
      return ChangeMissionResponse(False, "%s"%e)
    
    if res_rover.success:
      rospy.set_param('/base_station/Mission', req.mission)
      res_base = ChangeMissionResponse(True, res_rover.message)
    else:
      res_base = ChangeMissionResponse(False, res_rover.message)
              
    return res_base

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # driveCb():
  #    Callback for control msgs designated for DRIVE mode with xbox controller.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def driveCb(self, msg):

    global speed_change_flag
    rpm_limit   = rospy.get_param('rpm_limit', 0.3)
    steer_limit = rospy.get_param('steer_limit', 0.3)
    
    drive_msg = DriveCmd() # New drive command message
    
    # TODO define max RPM of motors and replace 50 with it
    drive_msg.rpm       =  50 * rpm_limit   * msg.axis_ly_val
    if drive_msg.rpm < 0:
    	drive_msg.steer_pct = - 100 * rpm_limit * msg.axis_rx_val 
    else:      
    	drive_msg.steer_pct = 100 * rpm_limit * msg.axis_rx_val 
    
    
    if speed_change_flag==False and msg.axis_dy_dwn != 0:
         speed_change_flag = True
         if msg.axis_dy_dwn == 1:
            rospy.set_param('rpm_limit',rpm_limit+0.1)
            rospy.loginfo("Up")
            self.max_speed_pub.publish(10)
            
         if msg.axis_dy_dwn == -1:
            if rpm_limit <= 0.0:
               rpm_limit = 0.0
               rospy.set_param('rpm_limit',0.0)
            else:
               self.max_speed_pub.publish(-10)
               rospy.set_param('rpm_limit',rpm_limit-0.1)
               rospy.loginfo("Down")
    elif msg.axis_dy_dwn == 0:
         speed_change_flag = False

    if rospy.get_param("/base_station/Mode") == "Drive":
         self.drive_cmd_pub.publish(drive_msg) # Send it


  def rightDriveCb(self, msg):

    rpm_limit   = rospy.get_param('rpm_limit', 0.3)
    steer_limit = rospy.get_param('steer_limit', 0.3)
    
    drive_msg = DriveCmd() # New drive command message

    msg_rpm = msg.axis_ly_val 
    msg_steer_pct = msg.axis_lx_val 
   
    #Adding in a deadzone
    if abs(msg_rpm)<0.3:
        msg_rpm = 0
    if abs(msg_steer_pct)<0.3:
        msg_steer_pct = 0

    drive_msg.rpm       =  50 * rpm_limit   * msg_rpm  
    drive_msg.steer_pct = 100 * steer_limit * msg_steer_pct
    
    if rospy.get_param("/base_station/Mode") == "Drive": 
      self.drive_cmd_pub.publish(drive_msg) # Send it


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # armCb():
  #    Callback for control msgs designated for ARM mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def armCb(self, msg):
    pass

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # drillCb():
  #    Callback for control msgs designated for DRILL mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def drillCb(self, msg):
    pass

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # rawCtrlCb():
  #    Callback for raw control messages from Xbox controller. Currently
  #    we don't care about STANDBY for this. In AUTO we can't touch 
  #    any controllers so the mode isn't handled.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def rawCtrlCb(self, msg):
    mode = rospy.get_param('base_station/Mode')
    drive_mode = rospy.get_param('base_station/drive_mode')
    
    if msg.but_y_trg == True:
	    rospy.loginfo("True")
	    rospy.set_param('base_station/drive_mode','XboxDrive')
    
    if   (drive_mode == 'XboxDrive' and mode == 'Drive'): 
      self.driveCb(msg)

    elif (mode == 'Arm'):
      self.armCb  (msg)

    elif mode is (mode == 'Drill'):
      self.drillCb(msg)


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # jpystoclCtrlCb():
  #    Callback for raw control messages from Xbox controller. Currently
  #    we don't care about STANDBY for this. In AUTO we can't touch 
  #    any controllers so the mode isn't handled.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

  def rightRawCtrlCb(self, msg):
    mode = rospy.get_param('base_station/Mode')
    drive_mode = rospy.get_param('base_station/drive_mode')
    mode = 'Drive'
    if msg.but_b_trg == True:
	    rospy.loginfo("True")
	    rospy.set_param('base_station/drive_mode','RightDrive')
    if   (drive_mode=='RightDrive' and mode=='Drive'):
        self.rightDriveCb(msg)
  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # initialiseState():
  #    Set the relevant parameters in the server for the initial rover
  #    state.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def initialiseState(self):
    state_vehicle = rospy.get_param('~Vehicle')
    rospy.set_param('Vehicle', state_vehicle) 

    rospy.set_param('/base_station/Mode', 'Standby')
    rospy.set_param('/base_station/Mission', '')

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def main():
  _ = BaseSync()
  rospy.set_param('base_station/drive_mode','XboxDrive')
  rate = rospy.Rate(10)
  hbeat_pub = rospy.Publisher('/heartbeat', Empty, queue_size=1)
  
  hbeat_loop_cnt = 0
  
  while not rospy.is_shutdown(): 
    if (hbeat_loop_cnt > 10):
        hbeat_msg = Empty()
        hbeat_pub.publish(hbeat_msg)
        hbeat_loop_cnt = 0
    
    hbeat_loop_cnt += 1

    rate.sleep()

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
