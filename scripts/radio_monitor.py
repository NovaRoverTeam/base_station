#!/usr/bin/env python

import rospy
import paramiko
import sys
import re
import signal

import warnings # Suppressing annoying warning in paramiko
warnings.filterwarnings("ignore", category=FutureWarning)

from base_station.msg import RadioStatus # Import custom msg

tmout = 1.5 # Timeout (in seconds) for ssh channel


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# ssh_connect():
#
#    Connect an SSH client to the SSH server running on the radio.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
def ssh_connect(host, user, pw, i):
  ssh = paramiko.SSHClient()
  ssh.load_system_host_keys()
  ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
  
  try:
    ssh.connect(host, username=user, password=pw, timeout=tmout) 
    rospy.loginfo("Connected SSH client to radio " + str(i) + " at " + host + ".")   
  except:
    pass
    
  return ssh
  

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# is_connected():
#
#    Check if the current SSH connection is active.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def is_connected(ssh):  
  try:
    ssh.exec_command('ls', timeout=tmout)
    return True    
  except:
    return False
    
    
#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# get_signal():
#
#    Read the signal strength from the radio.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def get_signal(ssh):   
  try:
    # Call mca-status to get radio information
    stdin, stdout, stderr = ssh.exec_command("mca-status | grep signal", timeout=tmout)
    
    # Process resulting string to get value of signal strength      
    signal_strength = [int(s) for s in re.findall(r'[-]?[\d]+', stdout.read())][0]    
    return signal_strength
    
  except:
    return None


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# get_n_wlan_cons():
#
#    Read the number of devices paired to the radio.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--  
def get_n_wlan_cons(ssh):   
  try:
    # Call mca-status to get radio information
    stdin, stdout, stderr = ssh.exec_command("mca-status | grep wlanConnections", timeout=tmout)
    
    # Process resulting string to get value of signal strength      
    n_wlan_cons = [int(s) for s in re.findall(r'[-]?[\d]+', stdout.read())][0]    
    return n_wlan_cons
    
  except:
    return None


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# main():
#
#    Main function.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
def main():
  rospy.init_node('radio_monitor')
  rate = rospy.Rate(0.3)
  
  pub = rospy.Publisher('radio_status', RadioStatus, queue_size=1)
  
  # Get radio authentication info from launch file
  hosts = [rospy.get_param("~radio_0/host"), rospy.get_param("~radio_1/host")]
  users = [rospy.get_param("~radio_0/user"), rospy.get_param("~radio_1/user")]
  pws   = [rospy.get_param("~radio_0/pw"),   rospy.get_param("~radio_1/pw")  ]
    
  ssh = [] # Connect to both radios via ssh
  for i in range(2):
    ssh.append(ssh_connect(hosts[i], users[i], pws[i], i))
  
  # Set up sigint handler to close ssh sessions when Ctrl+C pressed
  def signal_handler(sig, frame):
    ssh[0].close() 
    ssh[1].close()
    rospy.signal_shutdown("SIGINT")
    
  signal.signal(signal.SIGINT, signal_handler) # Register sigint handler
  
  while not rospy.is_shutdown():
  
    for i in range(2):
      # If ssh disconnected, reconnect and give error msg
      if not is_connected(ssh[i]): 
        rospy.loginfo("SSH client cannot connect to radio " + str(i) + ".")
        ssh[i] = ssh_connect(hosts[i], users[i], pws[i], i)
        
        msg = RadioStatus()
        msg.radio_id = i
        pub.publish(msg)    
	      
      # If ssh all gucci, grab and publish radio status information
      else:
        signal_strength = get_signal(ssh[i])
        n_wlan_cons = get_n_wlan_cons(ssh[i])
        
        if signal_strength is not None and n_wlan_cons is not None:
	        msg = RadioStatus()
	        msg.radio_id = i
	        msg.n_wlan_cons = n_wlan_cons
	        msg.signal = signal_strength
	        msg.ssh_active = True
	        pub.publish(msg) 
        else:
          rospy.loginfo("SSH timeout on radio " + str(i) + ".")      
    
    rate.sleep()


#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# Initialiser.
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..-- 
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

