#!/usr/bin/env python

import rospy
from nova_common.msg import *
from nova_common.srv import *
from base_station.msg import *

class StateManager:

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # __init__():
  #
  #    Initialise class.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
  def __init__(self):

    rospy.init_node('state_manager') 

    self.initialise_state() # Set initial parameter server values

    self.raw_ctrl_sub   = rospy.Subscriber(
        '/base_station/raw_ctrl', RawCtrl, self.raw_ctrl_cb)

    self.radio_stat_sub = rospy.Subscriber(
        '/base_station/radio_status', RadioStatus, self.radio_stat_cb)

    self.change_state_server = rospy.Service(
        '/base_station/change_state', ToggleStream, 
        self.handle_change_state)

    # Connectivity status for ROS radio
    self.radio_connected = False


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # handle_change_state():
  #
  #  Service server handler for changing rover state.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
  def handle_change_state(req):

    rospy.set_param(req.state, req.value)
    
    message = ("State " + req.state + " changed to " + req.value 
      + " successfully.")
    res = ChangeStateResponse(True, message)
    return res       


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # radio_stat_cb():
  #
  #    Callback for radio status messages to check connectivity.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def radio_stat_cb(self, msg):
    radio_ros = rospy.get_param('RadioROS')
  
    if msg.radio_id is radio_ros: # If receiving message from ROS radio
     
      if msg.ssh_active and msg.n_wlan_cons > 0: # Record status
        self.radio_connected = True
      else:
        self.radio_connected = False


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # drive_cb():
  #
  #    Callback for control msgs designated for DRIVE mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def drive_cb(self, msg):
    pass


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # arm_cb():
  #
  #    Callback for control msgs designated for ARM mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def arm_cb(self, msg):
    pass


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # drill_cb():
  #
  #    Callback for control msgs designated for DRILL mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def drill_cb(self, msg):
    pass


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # raw_ctrl_cb():
  #
  #    Callback for raw control messages from Xbox controller. Currently
  #    we don't care about STANDBY for this. In AUTO we can't touch 
  #    any controllers so the mode isn't handled.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def raw_ctrl_cb(self, msg):
    mode = rospy.get_param('BaseMode')

    if self.radio_connected:

      if   mode is 'Drive': 
        drive_cb(msg)

      elif mode is 'Arm':
        arm_cb(msg)

      elif mode is 'Drill':
        drill_cb(msg)


  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # initialise_state():
  #
  #    Set the relevant parameters in the server for the initial rover
  #    state, primarily the mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def initialise_state(self):
    # TODO set these from launch file
    #state_radio_ros = rospy.get_param('~radio_ros')

    state_vehicle = rospy.get_param('~Vehicle')

    rospy.set_param('Vehicle', state_vehicle) 
    rospy.set_param('RadioROS', 0) 
    rospy.set_param('BaseMode', 'Standby')


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Main():
#
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def main():
  state_manager = StateManager()
  
  while not rospy.is_shutdown(): 
    pass


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
