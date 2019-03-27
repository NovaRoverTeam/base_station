#!/usr/bin/env python

import rospy
from nova_common.msg import *
from nova_common.srv import *
from base_station.msg import *

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

    self.change_mode_server = rospy.Service(
      '/base_station/change_mode', ChangeMode, 
      self.handleChangeMode)

    self.change_mission_server = rospy.Service(
      '/base_station/change_mission', ChangeMode, 
      self.handleChangeMission)
        
    self.drive_cmd_pub = rospy.Publisher(
      '/core_rover/driver/drive_cmd', DriveCmd, 
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
        ChangeMode)
      res_rover = client(req.mode)
        
    except rospy.ServiceException, e:
      rospy.loginfo("Changing mission failed: %s"%e)
      return ChangeModeResponse(False, "%s"%e)
    
    if res_rover.success:
      rospy.set_param('/base_station/Mission', req.mode)
      res_base = ChangeModeResponse(True, res_rover.message)
    else:
      res_base = ChangeModeResponse(False, res_rover.message)
              
    return res_base

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # driveCb():
  #    Callback for control msgs designated for DRIVE mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def driveCb(self, msg):
    
    rpm_limit   = rospy.get_param('rpm_limit')
    steer_limit = rospy.get_param('steer_limit')
    
    drive_msg = DriveCmd() # New drive command message
    
    # TODO define max RPM of motors and replace 50 with it
    drive_msg.rpm       =  50 * rpm_limit   * msg.axis_ly_val   
    drive_msg.steer_pct = 100 * steer_limit * msg.axis_rx_val 
    
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

    if   (mode == 'Drive'): 
      self.driveCb(msg)

    elif (mode == 'Arm'):
      self.armCb  (msg)

    elif mode is (mode == 'Drill'):
      self.drillCb(msg)

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
  
  rate = rospy.Rate(0.1)
  while not rospy.is_shutdown(): 
    rate.sleep()

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
