#!/usr/bin/env python

import rospy
from nova_common.msg import *
from nova_common.srv import *
max_speed = 0.3
max_steer = 0.3
speed_change_flag = False
steer_change_flag = False
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
    global max_speed
    global max_steer
    global speed_change_flag
    # TODO define max RPM of motors and replace 50 with it
    drive_msg.rpm       =  50 * msg.axis_ly_val  * max_speed
    drive_msg.steer_pct = 100 * msg.axis_rx_val  * max_steer
    
    if speed_change_flag==False and msg.axis_dy_dwn != 0:
         speed_change_flag = True
         if msg.axis_dy_dwn == 1:
            max_speed += 0.1
            rospy.loginfo("Up")
         if msg.axis_dy_dwn == -1:
            max_speed -= 0.1
            if max_speed < 0.0:
               max_speed = 0.0
            rospy.loginfo("Down")
    elif msg.axis_dy_dwn == 0:
         speed_change_flag = False
    
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
