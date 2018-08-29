#!/usr/bin/env python

import rospy


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def main():
  rospy.init_node('camera_client')
  rate = rospy.Rate(1)
    
  # Set up custom sigint handler
  def signal_handler(sig, frame):
    rospy.signal_shutdown("SIGINT")
    
  signal.signal(signal.SIGINT, signal_handler) # Register sigint handler
  
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
