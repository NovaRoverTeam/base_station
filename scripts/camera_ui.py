#!/usr/bin/env python

# Import needed libraries
import gi, sys, os, traceback
from PyQt5.QtWidgets import *

# Get specific versions
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('Gtk', '3.0')

from gi.repository import Gst, GObject, Gtk
from gi.repository import GdkX11, GstVideo


# Initialise the GStreamer library with arguments
Gst.init(sys.argv)


# Set up application
app = QApplication([])
window = QWidget()
layout = QVBoxLayout()



# Create a message pipeline function, for outputting messages to the terminal
def on_message(bus, message, loop):
	mtype = message.type
	"""
	Gstreamer Message Types and how to parse
        https://lazka.github.io/pgi-docs/Gst-1.0/flags.html#Gst.MessageType
	"""
	if mtype == Gst.MessageType.EOS:
		print("End of stream")
        
	elif mtype == Gst.MessageType.ERROR:
		err, debug = message.parse_error()
		print(err, debug)

	elif mtype == Gst.MessageType.WARNING:
		err, debug = message.parse_warning()
		print(err, debug)       
        
	return True 




pipelines = []
buses = []


def getGSTCommand (_port):
	return ("udpsrc port={} ! queue ! application/x-rtp, encoding-name=JPEG, payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink".format(_port))



# Set GST command
streams = (getGSTCommand(5000), getGSTCommand(5001), getGSTCommand(5002), getGSTCommand(5003),
"videotestsrc ! videoconvert ! ximagesink")

# Set Feed names
feedNames = ('Stereo Camera', 'Arm Camera', 'Black Foscam', 'White Foscam', 'Test Feed')


# Set Feed Playing States
feedPlaying = [False, False, False, False, False]


# Create a pipeline from each command
for stream in streams:
	# Create pipeline
	pipeline = Gst.parse_launch(stream)
	bus = pipeline.get_bus()  

	# Allow bus to emit messages to main thread
	bus.add_signal_watch()

	# Add handler to specific signal
	bus.connect("message", on_message, None)

	pipelines.append(pipeline)
	buses.append(bus)



# Key Variables for Cameras
port = 5000
cameraIndex = 0




# Set up GUI elements:
cb_camera = QComboBox()
cb_camera.addItems(feedNames)
label_port = QLabel('Port: ')
txt_port = QLineEdit(str(port))

btn_view = QPushButton('View Stream')
btn_stop = QPushButton('Close Stream')



# Setup function when camera change is altered
def cb_camera_change(i):
	global cameraIndex, port

	# Set key variables
	if i == 0: # Stereo Cam
		port = 5000
		txt_port.setText(str(port))
	elif i == 1: # Arm Cam
		port = 5001
		txt_port.setText(str(port))
	elif i == 2: # Black Fos Cam
		port = 5002
		txt_port.setText(str(port))
	elif i == 3: # White Fos Cam
		port = 5003
		txt_port.setText(str(port))
	elif i == 4: # Test Feed
		port = 0
		txt_port.setText(str(port))

	# Update camera index	
	cameraIndex = i

	# Change the buttons from being enabled or not
	if feedPlaying[cameraIndex] == True:
		btn_view.setEnabled(False)
		btn_stop.setEnabled(True)
	else:
		btn_view.setEnabled(True)
		btn_stop.setEnabled(False)



# Add camera widgets to layout
# Camera Combo Box
cb_camera.currentIndexChanged.connect(cb_camera_change)
layout.addWidget(cb_camera)

# Port text
layout.addWidget(label_port)
layout.addWidget(txt_port)





# Button for viewing stream
def on_btn_view():
	global cameraIndex
	
	# Turn pipeline on
	pipelines[cameraIndex].set_state(Gst.State.PLAYING)
	feedPlaying[cameraIndex] = True
	print('Attempting to start {} feed...'.format(feedNames[cameraIndex]))

	# Update window
	cb_camera_change(cameraIndex)


btn_view.clicked.connect(on_btn_view)
layout.addWidget(btn_view)



# Button for stopping stream
def on_btn_stop():
	global cameraIndex

	# Turn pipeline off
	pipelines[cameraIndex].set_state(Gst.State.NULL)
	feedPlaying[cameraIndex] = False
	print('Attempting to stop {} feed...'.format(feedNames[cameraIndex]))

	# Update window
	cb_camera_change(cameraIndex)

btn_stop.clicked.connect(on_btn_stop)
layout.addWidget(btn_stop)

# Start with stop button not enabled
btn_stop.setEnabled(False)



# Button for quitting application
btn_quit = QPushButton('Quit')
def on_btn_quit():
	isRunning = False
	window.hide()
	print('Closing Program.')
	exit()

btn_quit.clicked.connect(on_btn_quit)
layout.addWidget(btn_quit)





# Run all pipelines at once
for pipeline in pipelines:
	pipeline.set_state(Gst.State.NULL)

# Loop until stopping
isRunning = True




# Run application
window.setLayout(layout)
window.show()
app.exec_()


isRunning = False

print("Running GStreamer camera feed window. Type 'help' or 'h' for more information.")

# While program is running, watch for commands
while isRunning:
	# Get user command
	command = str(input('\nEnter a new command: ')).lower()
	
	# Help command
	if command == 'h' or command == 'help':
		print(
'\n****************************' +
'\n    Program Command List    ' +
'\n****************************' +
"\nExit Program:\t'e', 'exit'" +
"\nHelp:\t\t'h', 'help'" +
"\nStart Feed:\t's', 'start'" +
"\nStop Feed:\t'x', 'stop'" +
'\n')
		
	# Exit command
	if command == 'e' or command == 'exit':
		isRunning = False
		print('Closing Program.')


	# Start Feed command
	if command == 's' or command == 'start':
		print('Current Feed List:')
		for idx, name in enumerate(feedNames):
			print(str(idx) + '\t' + name)
		
		# Attempt to get feed value
		try:
			feedVal = int(input('Select a feed to Start: '))
			pipelines[feedVal].set_state(Gst.State.PLAYING)
			print('Attempting to start feed {}...'.format(feedVal))
		except:
			print('ERROR: Invalid Feed Entry.')
		
		
	# Stop Feed command
	if command == 'x' or command == 'stop':
		print('Current Feed List:')
		for idx, name in enumerate(feedNames):
			print(str(idx) + '\t' + name)
		
		# Attempt to get feed value
		try:
			feedVal = int(input('Select a feed to Stop: '))
			pipelines[feedVal].set_state(Gst.State.NULL)
			print('Attempting to stop feed {}...'.format(feedVal))
		except:
			print('ERROR: Invalid Feed Entry.')




	





















