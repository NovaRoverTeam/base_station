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
pub = rospy.Publisher('/core_rover/driver/drive_cmd', DriveCmd, queue_size=10) #Create publisher
control_socket = IOBlueprint('/')

@routes.route('/control')
def control():
    msg = Int32()
    msg.data = 16
    #pub.publish(msg)
    return render_template("control.html")

@control_socket.on('control/drive_cmd')
def change_drive_control(message):
    msg = DriveCmd()
    msg.rpm = message['rpm']
    msg.steer_pct = message['steer_pct']
    pub.publish(msg)
    #set_param('drive_control',data)

@control_socket.on('control/speed_setting')
def change_drive_control(message):
    rospy.set_param('speed_setting',message['data'])

@control_socket.on('control/turn_setting')
def change_drive_control(message):
    rospy.set_param('turn_setting',message['data'])

@control_socket.on('control/drive_mode')
def change_drive_control(message):
    msg = Int32()
    msg.data = 17
    #pub.publish(msg)
    #set_param('drive_control',data)
    print('lmao')
    emit('response',{'data':'lol'})

@control_socket.on('control/init')
def control_init():
    speed = 50
    turning = 50
    speed = rospy.get_param('speed_setting')
    turning = rospy.get_param('turn_setting')
    emit('control/response',{'speed':speed,'turn':turning})
