#!/usr/bin/env python

# Import needed libraries
import gi, sys, os, traceback

# Get specific versions
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('Gtk', '3.0')

from gi.repository import Gst, GObject, Gtk
from gi.repository import GdkX11, GstVideo



# Initialise the GStreamer library with arguments
Gst.init(sys.argv)


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






# Run all pipelines at once
for pipeline in pipelines:
	pipeline.set_state(Gst.State.NULL)

# Loop until stopping
isRunning = True

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




	
