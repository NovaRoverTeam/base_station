#! /usr/bin/env python

#Flask Depencies
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
#Import Blueprints
from routes import *
#Ros Dependencies
import os
import rospy
import threading

project_root = os.path.dirname(__file__) #Locate directory
template_path = os.path.join(project_root, '../resources/server/templates') #Locate Folder for templates
static_path = os.path.join(project_root,'../resources/server/static')
threading.Thread(target=lambda: rospy.init_node('flask-ros', disable_signals=True)).start() #Used to create a node in a non-interfering way
app = Flask(__name__,template_folder=template_path, static_folder=static_path) #Create app and specify template 
socketio = SocketIO(app) #Create the main socket instance

app.config['SECRET_KEY'] = 'secret!'
app.register_blueprint(routes) #Register blueprint pages to main app

#Register socketio instances in each blueprint
from routes.control import control_socket
from routes.surveillance import surveillance_socket
from routes.radio import radio_socket
from routes.arm import arm_socket
#from routes.home import home_socket 

control_socket.init_io(socketio)
surveillance_socket.init_io(socketio)
radio_socket.init_io(socketio)
arm_socket.init_io(socketio)
#home_socket.init_io(socketio)

def init(): #initialize initial parameters
    rospy.set_param('speed_setting',50)
    rospy.set_param('turn_setting',50)

if __name__ == '__main__':
    init()
    socketio.run(app,host='0.0.0.0',port=5000,debug=True) #Run it publicly on local address on port 5000

