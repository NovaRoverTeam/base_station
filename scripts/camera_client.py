#!/usr/bin/env python

import rospy
import signal

import pygst          # GStreamer includes
pygst.require("0.10")
import gst
import pygtk
import gtk

from nova_common.srv import * # Import custom msg
from nova_common.msg import CameraStatus


class ServiceHandler:

    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # __init__():
    #
    #    Initialise class.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--    
    def __init__(self):
      rospy.init_node('camera_client') 
      
      self.view_server = rospy.Service('/base_station/toggle_cam_view', ToggleStream, 
                                         self.handle_toggle_cam_view)
      self.stream_server = rospy.Service('/base_station/toggle_stream', ToggleStream, 
                                  self.handle_toggle_stream)
      self.camstat_sub = rospy.Subscriber('/core_rover/camera_status', CameraStatus, 
                                            self.camstat_cb, queue_size=1)                                  
                                  
      self.streams = [None]*5
      self.cur_con = [False]*5
      self.port_base = 10720


    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # camstat_cb():
    #
    #    Callback for monitoring camera streaming status.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
    def camstat_cb(self, msg):
      self.cur_con[msg.cam_id] = msg.streaming 
      
      if self.streams[msg.cam_id] is not None and msg.streaming is False:
        self.streams[msg.cam_id].set_state(gst.STATE_NULL)


    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # handle_toggle_cam_view():
    #
    #    Service server handler for toggling camera streams on and off.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def handle_toggle_cam_view(self, req):      
      res = ToggleStreamResponse()
      
      if self.cur_con[req.cam_id]: # If stream is active
        
        if req.on: # We want to show cam view   
        
          if self.streams[req.cam_id] is None:
            self.streams[req.cam_id] = gst.parse_launch('udpsrc port=' 
              + str(self.port_base + req.cam_id) + ' ! application/x-rtp, '
              + 'encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec '
              + '! autovideosink')
          
          self.streams[req.cam_id].set_state(gst.STATE_READY)    
          self.streams[req.cam_id].set_state(gst.STATE_PLAYING)
               
        else:      # We want to hide cam view
          self.streams[req.cam_id].set_state(gst.STATE_NULL)
          
      else: 
        res.message = ("Stream for camera " + str(req.cam_id) + " is "
          "currently inactive")
        res.success = False
              
      res.success = self.cur_con[req.cam_id]        
      
      return res
        
        
    #--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
    # handle_toggle_stream():
    #
    #    Service server handler for starting and stopping camera streams.
    #--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
    def handle_toggle_stream(self, req):   
      rospy.wait_for_service('connect_stream')
      
      try:          
        client = rospy.ServiceProxy('/core_rover/connect_stream', ToggleStream)          
        res = client(req.cam_id, req.on)      
          
      except rospy.ServiceException, e:
        rospy.loginfo("camera client here, cam view request failed.")
        rospy.loginfo("Service call failed: %s"%e)             
      
      return res
        

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def main():

  sh = ServiceHandler()
  rate = rospy.Rate(1)
  
  # Set up custom sigint handler
  def signal_handler(sig, frame):
    try:
      gtk.main_quit()               # Shut down gstreamer
    except RuntimeError:
      rospy.loginfo("Forcing gstreamer shutdown.")
      rospy.signal_shutdown("SIGINT") # Shut down ROS   
      sys.exit(1)
  
  signal.signal(signal.SIGINT, signal_handler) # Register sigint handler
    
  while not rospy.is_shutdown():       
    rate.sleep()

  # Set up custom sigint handler
  #def signal_handler(sig, frame):
  #  rospy.signal_shutdown("SIGINT")
  #  
  #signal.signal(signal.SIGINT, signal_handler) # Register sigint handler
  

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    start = main()   
    gtk.main()
    
  except rospy.ROSInterruptException:
    pass
    
