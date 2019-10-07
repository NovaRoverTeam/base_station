#! /usr/bin/env python

#Flask Depencies
from flask import Flask, render_template, session
from flask_socketio import SocketIO, emit
from flask_app import socketio
from io_blueprint import IOBlueprint
from . import routes
#Ros Dependencies
import os
import rospy
import threading
from std_msgs.msg import Int32
from nova_common.msg import *
arm_l_pub = rospy.Publisher('/base_station/ljs_raw_ctrl', RawCtrl, queue_size=10) #Create publisher
arm_r_pub = rospy.Publisher('/base_station/rjs_raw_ctrl', RawCtrl, queue_size=10)
arm_socket = IOBlueprint('/')

@routes.route('/arm')
def arm():
    msg = Int32()
    msg.data = 16
    #pub.publish(msg)
    return render_template("arm.html")


@arm_socket.on('arm/arm_cmd')
def change_arm_control(message):
    msg_l = RawCtrl()
    msg_r = RawCtrl()
    
    msg_l.axis_lx_val = message['llx']
    msg_l.axis_ly_val = message['lly']
    msg_l.trig_l_val = message['ltl']
    msg_l.trig_r_val = message['ltr']

    msg_r.axis_lx_val = message['rlx']
    msg_r.axis_ly_val = message['rly']
    msg_r.trig_l_val = message['rtl']
    msg_r.trig_r_val = message['rtr']

    arm_l_pub.publish(msg_l)
    arm_r_pub.publish(msg_r)
    #set_param('drive_control',data)

@arm_socket.on('arm/speed_setting')
def change_arm_control(message):
    rospy.set_param('speed_setting',message['data'])

@arm_socket.on('arm/turn_setting')
def change_drive_control(message):
    rospy.set_param('turn_setting',message['data'])

@arm_socket.on('arm/drive_mode')
def change_arm_control(message):
    msg = Int32()
    msg.data = 17
    #pub.publish(msg)
    #set_param('drive_control',data)
    print('lmao')
    emit('response',{'data':'lol'})

def arm_init():
    speed = 50
    turning = 50
    speed = rospy.get_param('speed_setting')
    turning = rospy.get_param('turn_setting')
    emit('control/response',{'speed':speed,'turn':turning})
