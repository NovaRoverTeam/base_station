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

home_socket = IOBlueprint('/')  # Create new socketio instance for blueprint

@routes.route('/')
def home():
    return render_template("index.html")