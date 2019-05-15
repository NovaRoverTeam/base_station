# This script will create a map with GSP coordinates and available for Extreme Retrieval Task
# It must gather data points from the base station, via ethernet or other connection
# Simple map plotting can be used, but future updates could include a Google Maps background image
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.transforms as mtransforms
import socket, time
from matplotlib.patches import Ellipse


# Create window ready
#plt.rcParams['toolbar'] = 'None' # Turn off toolbar
fig = plt.figure('Rover Location', figsize=(5.5, 5)) # Create figure
plt.rc('font', **{'size':8}) # Change font size

# Get UDP streams of current device connection
UDP_IP = '192.168.1.8'
UDP_PORT = 6000

# Create data connection sink
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Plotting Data Variables
coordinates = (0.0, 0.0) # GPS Coordinates
pastCoordinates = [] # List of all past coordinates
waypoints = []
yawDirfactor = 0.00002 # How far to move direction marker from coordinates
firstCoordinate = (0.0, 0.0) # The first GSP coordinate (for creating axis)

# Rotation Variables
yaw = 0.0 # Current Yaw
pitch = 0.0
roll = 0.0

# Constant conversion points
latToM = 110000
longToM = 80000


# This function takes in a yaw and a length and creates a coordinate of that angle away from the
# 			base point. For example, from (0, 0), input of yaw = 45 and length = 2 would return the
#				coordinate (Sqrt(2), Sqrt(2)).
def yawDirection (_yaw, _length):
	# Get angle in radians
	rad = _yaw * np.pi / 180.0
	# Calculate direction
	direction = (_length * np.sin(rad), _length * np.cos(rad))
	return direction


# Set last GPS time
time_last_GPS = 0

while True:

	data, addr = sock.recvfrom(1024)
	#data = raw_input('Data: ')#'G 32.1127 131.2338 127'
	print( "Receieved Message: ", data)
	
	# For multiline data sets
	for line in data.split('!'):
	
		# Get the command of the data stream
		command = line.replace(' ','')[0]
		
		# GPS Coordinates
		if command == 'G':
			values = line.strip().split(' ')
			coordinates = (float(values[1]), float(values[2]))
			yaw = float(values[3])
			
			# If this is the first coordinate
			if firstCoordinate == (0.0, 0.0):
				firstCoordinate = coordinates
				
			# Add history of GPS
			# If 5 seconds has elapsed since last saved GPS
			if time.time() - time_last_GPS > 5.0:
				# Update the time frame
				time_last_GPS = time.time()
				# Add the coordinate to the list
				pastCoordinates.insert(0, coordinates)
				
			# If pitch and roll values are present
			if len(values) > 4:
				pitch = values[4]
				roll = values[5]
		
		# New Waypoint plot
		elif command == 'W':
			values = line.strip().split(' ')
			waypoints.insert(0, (float(values[1]), float(values[2])))
		
		# Otherwise skip this loop frame
		else:
			continue

	# Clear the current plot
	plt.cla()
	
	plt.autoscale(False)
	
	
		
	# Plot the past GPS coordinates
	# Get a list of X and Y coordinates (only first 100 points)
	x = [coord[1] for coord in pastCoordinates[0:100]]
	y = [coord[0] for coord in pastCoordinates[0:100]]
	plt.plot(x, y, linewidth=1.0)
	
	
	
	# Plot the current GPS location
	plt.scatter(coordinates[1], coordinates[0], color='cyan', s=30)
	
	# Plot the direction marker
	yaw_dir = yawDirection(yaw, yawDirfactor)
	plt.scatter(yaw_dir[0] + coordinates[1], yaw_dir[1] + coordinates[0], color='red', s=10)
	
	
	
	# Plot the waypoints
	for idx, coord in enumerate(waypoints):
		# Set Alpha position
		if idx < 10:
			alpha = 1.0/(float)(idx + 1)
		else:
			alpha = 0.1
		
		# Set middle colour
		if idx == 0:
			facecolors = 'none'
		else:
			facecolors = 'lime'
			
		plt.scatter(coord[1], coord[0], color='lime', alpha=alpha, facecolors=facecolors)
	
	
	# Adjust the axis and graph
	ax = plt.gca()
	ax.set_facecolor('xkcd:black')
	ax.grid(alpha=0.2)
	
		
	# Adjust the scale to fit the GPS coordinate
	# 160 Meter diameter latitude and longitude
	diameterLat = (1.0 / latToM)
	diameterLong = (1.0 / longToM)
	axisInM = 100.0 # The size of half the screen in meters
	ax.set_xlim(coordinates[1] - diameterLong * axisInM, coordinates[1] + diameterLong * axisInM)
	ax.set_ylim(coordinates[0] - diameterLat * axisInM, coordinates[0] + diameterLat * axisInM)
	
	# Create a ellipse from the GPS coordinates with different intervals (in meters)
	eIntervals = (20.0, 50.0, 100.0)
	ellipses = [Ellipse(xy = (coordinates[1], coordinates[0]), width = diameterLong * eInt, height = diameterLat * eInt, facecolor = 'none', edgecolor = 'w', lw = 0.5, alpha = 0.5) for eInt in eIntervals]
	
	# Add the ellipses to the plot
	for ellipse in ellipses:
		ax.add_artist(ellipse)
	
	# Remove white space padding
	fig.tight_layout()
	
	# Draw the plot to the screen
	plt.pause(0.01)
	plt.draw()




	# Create rotation cube
	




# Actual Field Test Data
'G -37.661300 145.36949 15 ! W -37.661350 145.369090 ! W -37.661590 145.398903 ! W -37.661696 145.368871 ! W -37.661766 145.368984 ! W -37.661769 145.369721 ! W -37.661532 145.369992 ! W -37.661099 145.370272 ! W -37.660983 145.369617 ! W -37.661035 145.369337'

# Access token for using maps [IGNORE]
mapbox_access_token = 'novarovpk.eyJ1Ijoibm92YXJvdmVyIiwiYSI6ImNqdmVvYjM5dDBxbmM0NHBsdm43NGMzZ3gifQ.oCC7YN-_Ege9GOD22yzOLw'

















