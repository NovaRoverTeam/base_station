#!/usr/bin/env python

# Import needed libraries
import gi, sys, os, traceback
from PyQt5.QtWidgets import *
from PyQt5 import QtGui

# Get specific versions
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('Gtk', '3.0')

from gi.repository import Gst, GObject, GstVideo


# Initialise the GStreamer library with arguments
Gst.init(sys.argv)


# Set up application
app = QApplication([])
window = QWidget()
window.setGeometry(0, 0, 500, 0)

# Create layouts
layout_L = QGridLayout() # Left column
layout_R = QHBoxLayout() # Right column
layout = QGridLayout() # Main layout
layout.setSpacing(10)



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


def getGSTCommand (_port, _flip):
	if _flip:
		return ("udpsrc port={} ! queue ! application/x-rtp, encoding-name=JPEG, payload=26 ! rtpjpegdepay ! jpegdec ! videoflip method=rotate-180 ! autovideosink".format(_port))

	else:
		return ("udpsrc port={} ! queue ! application/x-rtp, encoding-name=JPEG, payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink".format(_port))




####################################
# CAMERA VARIABLES
####################################

# Ports for each camera
cam_ports = []#(5000, 5002, 5003, 5021, 5022, 5023, 5024, 5025, 5001, 5033, 5031, 5032)
flipFeeds = []#(False, False, False, True, False, False, False, False, False, False, False, False)

# Set GST command
streams = []
# Set Feed names
feedNames = []#('Stereo Camera', 'Black Foscam', 'White Foscam', 'Telescopic Camera 1', 'Telescopic Camera 2', 'Telescopic Camera 3', 'Telescopic Camera 4', 'Telescopic Camera 5', 'Arm Stereo Camera', 'Real Sense', 'USB 3.0 Camera 1', 'USB 3.0 Camera 2')


# Set Feed Playing States
feedPlaying = []

# Reads the camera CSV file
with open('../../core_rover/scripts/cameras.csv', 'r') as file:
  content = file.readlines()
  attributes = content[0].replace('\n','').split(",")
  
  # Loop through each camera line, after the header line
  if (len(content) > 1):
    for camera in content[1:]:
      # Create a dictionary for the data
      data_dict = {}
      data = camera.replace('\n','').split(",")
      for i in range(0, len(data)):
        data_dict[attributes[i]] = data[i]
      
      # Set variables
      cam_ports.append(data_dict['Port'])
      flipFeeds.append(False)
      feedNames.append(data_dict['ID'])
      feedPlaying.append(False)
      streams.append(getGSTCommand(data_dict['Port'], False))

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
cameraIndex = 0
port = cam_ports[cameraIndex]




# Set up GUI elements:
cb_camera = QComboBox()
cb_camera.addItems(feedNames)
label_port = QLabel('Port: ')
txt_port = QLineEdit(str(port))

btn_view = QPushButton('View Stream')
btn_stop = QPushButton('Close Stream')



# Returns the status of the cameras as a text
def camera_status():
	text = ''

	# For each camera
	for idx, camera in enumerate(feedNames):
		# Add camera name
		text += '\n{}  [{}]'.format(camera, cam_ports[idx])

		# Get camera playing status
		if feedPlaying[idx]:
			camPlaying = "Playing"
		else:
			camPlaying = "Stopped"
		text += '\n\tStatus: {}'.format(camPlaying)

		# Get port of camera
		text += '\n'#'\n\tPort: {}\n'.format(cam_ports[idx])
	return text

label_status = QLabel(camera_status())
label_status.setStyleSheet('QLabel {font-size: 8pt;}')
layout_R.addWidget(label_status)




# Setup function when camera change is altered
def cb_camera_change(i):
	global cameraIndex, port

	# Set key variables
	port = cam_ports[i]
	txt_port.setText(str(port))	

	# Update camera index	
	cameraIndex = i

	# Change the buttons from being enabled or not
	if feedPlaying[cameraIndex] == True:
		btn_view.setEnabled(False)
		btn_view.setStyleSheet('QPushButton {color:grey;}')
		btn_stop.setEnabled(True)
		btn_stop.setStyleSheet('QPushButton {color:red;}')
	else:
		btn_view.setEnabled(True)
		btn_view.setStyleSheet('QPushButton {color:green;}')
		btn_stop.setEnabled(False)
		btn_stop.setStyleSheet('QPushButton {color:grey;}')

	# Update camera status
	label_status.setText(camera_status())


# Add camera widgets to layout
# Camera Combo Box
cb_camera.currentIndexChanged.connect(cb_camera_change)
layout_L.addWidget(cb_camera,0,0)

# Port text
txt_port.setEnabled(False)
layout_L.addWidget(txt_port,1,0)


# Button for viewing stream
def on_btn_view():
	global cameraIndex
	
	# Turn pipeline on
	pipelines[cameraIndex].set_state(Gst.State.PLAYING)
	feedPlaying[cameraIndex] = True
	print('Attempting to start {} feed...'.format(feedNames[cameraIndex]))

	# Update window
	cb_camera_change(cameraIndex)

btn_view.setStyleSheet('QPushButton {color:green;}')
btn_view.clicked.connect(on_btn_view)
layout_L.addWidget(btn_view, 3, 0)



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
layout_L.addWidget(btn_stop, 4, 0)

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
layout_L.addWidget(btn_quit, 5, 0)





# Run all pipelines at once
for pipeline in pipelines:
	pipeline.set_state(Gst.State.NULL)

# Loop until stopping
isRunning = True




# Run application
layout.addLayout(layout_L, 0, 0)
layout.addLayout(layout_R, 0, 1)
window.setLayout(layout)
window.setWindowTitle('Camera Stream Viewer')
#window.setWindowIcon(QtGui.QIcon('pythonlogo.png'))
window.show()
app.exec_()


isRunning = False











###########################################################
################## CONSOLE COMMANDS ONLY ##################
###########################################################

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




	





















