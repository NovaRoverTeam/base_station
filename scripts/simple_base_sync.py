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

    rospy.init_node('simple_base_sync') 

    self.raw_ctrl_sub   = rospy.Subscriber(
      '/base_station/xbox_raw_ctrl', RawCtrl, self.driveCb)
        
    self.drive_cmd_pub = rospy.Publisher(
      '/core_rover/driver/drive_cmd', DriveCmd, 
        queue_size=1)

  #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
  # driveCb():
  #    Callback for control msgs designated for DRIVE mode.
  #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
  def driveCb(self, msg):
    
    drive_msg = DriveCmd() # New drive command message
    
    # TODO define max RPM of motors and replace 50 with it
    drive_msg.rpm       =  50 * msg.axis_ly_val   *0.3
    drive_msg.steer_pct = 100 * msg.axis_rx_val  * 0.3
    
    self.drive_cmd_pub.publish(drive_msg) # Send it


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
