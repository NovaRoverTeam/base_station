from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import numpy as np
import socket, sys
from itertools import product, combinations

# Set figures
plt.rcParams['toolbar'] = 'None' # Turn off toolbar
fig = plt.figure('Rotation of Rover', figsize=(3, 3)) # Create figure
ax = fig.gca(projection='3d') # Get 3D axes
ax.format_coord = lambda x, y: '' # Turn off coordinates


# Get UDP streams of current device connection
UDP_IP = '192.168.1.8'
UDP_PORT = 6001

# Use socket network values? If not, manually enter data
if len(sys.argv) > 1 and str(sys.argv[1]).lower() == 'f':
	isSocket = False
else:
	isSocket = True
	
# Show output 3D map or not
if len(sys.argv) > 1 and str(sys.argv[1]).lower() == 't':
	showGraph = False
else:
	showGraph = True


# Create data connection sink
if isSocket:
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind((UDP_IP, UDP_PORT))



# Rotation Variables
yaw = 0.0
pitch = 0.0
roll = 0.0





##########################################
############ MATRIX FUNCTIONS ############
##########################################

# Converts a yaw value to a matrix
def matrix_YAW(_y):
	_y *= np.pi / 180.0
	return [[np.cos(_y), np.sin(_y), 0], [-np.sin(_y), np.cos(_y), 0], [0, 0, 1.0]]

# Converts a pitch value to a matrix
def matrix_PITCH(_p):
	_p *= np.pi / 180.0
	return [[1.0, 0, 0], [0, np.cos(_p), np.sin(_p)], [0, -np.sin(_p), np.cos(_p)]]

# Converts a roll value to a matrix
def matrix_ROLL(_r):
	_r *= np.pi / 180.0
	return [[np.cos(_r), 0, -np.sin(_r)], [0, 1.0, 0], [np.sin(_r), 0, np.cos(_r)]]
	
# Defines a matrix multiplication operation
def matrixMult(matrixA, matrixB):

	# Matrix A will be a m x r matrix
	# Matrix B will be a r x n matrix
	_m = len(matrixA)
	_r = len(matrixA[0])
	_n = len(matrixB[0])
	
	# If the matrices dont match up, return nothing
	if len(matrixB) != _r:
		print('Invalid Matrix Sizes')
		return None
		
	# Resulted matrix
	result = []
	
	# Loop through all rows
	for i in range(0, _m):	
		# Add a new row element
		result.append([])
	
		# Loop through all columns
		for j in range(0, _n):	
			# Create a new column element
			_sum = 0.0

			# Calculate the value at that cell
			for k in range(0, _r):
				_sum += matrixA[i][k] * matrixB[k][j]
				
			# Add the sum to the result matrix
			result[i].append(_sum)
			
	return result




# Set the Euler vectors for X, Y, Z
e_1 = [[1.0], [0], [0]]
e_2 = [[0], [1.0], [0]]
e_3 = [[0], [0], [1.0]]





######################################
############ MAIN PROCESS ############
######################################


# Loop through for each input value
while (True):
	# Socket data
	if isSocket:
		data, addr = sock.recvfrom(1024)
		val = data.strip().split(' ')
		
	else:
		# Get current values, with a split value
		val = raw_input('Enter values: ').strip().split(' ')
	
	# Run if there are three numbers (yaw, pitch, roll)
	if len(val) == 3:
		# Set the three transforms
		yaw = float(val[0])
		pitch = float(val[1])
		roll = float(val[2])
		
		print('YAW: {}, PITCH: {}, ROLL: {}'.format(yaw, pitch, roll))
		
	# Otherwise cancel the loop
	else:
		print('Invalid rotations entered')
		continue
		
	# Hide the graph if not showing
	if not showGraph:
		continue
	
	# Clear the plot and set the axes
	plt.cla()
	ax.set_aspect("equal")
	ax.set_autoscale_on(False)
	ax.set_xlim3d(-1, 1)
	ax.set_ylim3d(-1, 1)
	ax.set_zlim3d(-1, 1)

	# Calculate the transform matrices for each rotation
	m_yaw = matrix_YAW(yaw)
	m_pitch = matrix_PITCH(pitch)
	m_roll = matrix_ROLL(roll)
	

	# Calculate the total rotation matrix
	# ROT = ROLL * PITCH * YAW
	m_rotation = matrixMult(matrixMult(m_roll, m_pitch), m_yaw)

	# Calculate the new vectors by multiplying euler vectors with transformation matrix
	eulers = (e_1, e_2, e_3)
	
	# List the 8 points on the trapezoid object
	trapezoid_pts = ([[0.5], [1.0], [-0.5]], [[-0.5], [1.0], [-0.5]], [[-0.5], [-1.0], [-0.5]], [[0.5], [-1.0], [-0.5]], [[0.25], [0.5], [0.5]], [[-0.25], [0.5], [0.5]], [[-0.25], [-0.5], [0.5]], [[0.25], [-0.5], [0.5]])
	
	# Determine the 8 point locations once the matrix rotation is applied
	ROT_trap_pts = []
	for vec in trapezoid_pts:
		trans = matrixMult(m_rotation, vec)
		ROT_trap_pts.append((trans[0][0], trans[1][0], trans[2][0]))
		
	# List of each of the 6 faces, via the four vertex points
	ROT_trap_verts = [[0,1,5,4], [1,2,6,5], [2,3,7,6], [3,0,4,7], [4,5,6,7], [0,3,2,1]]
	
	# List of colors of the faces
	trap_colors = ['green', 'orange', 'lime', 'red', 'blue', 'cyan']
	
	# For each of the faces
	for idxF, face in enumerate(ROT_trap_verts):
		# Zip the vertices together into one array
		vert = [ROT_trap_pts[idx] for idx in face]

		# Create a quad and add i to the axis
		quad = Poly3DCollection([vert])
		quad.set_color(trap_colors[idxF])
		ax.add_collection3d(quad)
	
	# New vectors for plotting the lines
	newVectors = [matrixMult(m_rotation, vector) for vector in eulers]

	# Plot origin lines
	for v in eulers:
		ax.plot3D([0, 2 *v[0][0]], [0, 2 * v[1][0]], [0, 2 *v[2][0]], color='black', linewidth=1)

	# Loop through each rotation vector
	for idx, v in enumerate(newVectors):
		# Calculate the multiple of the rotation matrix with the euler vector
		colors = ['r', 'g', 'b']
		colors.extend(['black']*(len(newVectors) - 3))
		
		# Plot a line from origin to new point
		ax.plot3D([0, v[0][0]], [0, v[1][0]], [0, v[2][0]], color=colors[idx], linewidth=5)

	
	# Draw center sphere
	ax.scatter(0,0,0, color='black', s=100)	

	# Remove white space padding
	fig.tight_layout()
	
	# Draw the plot in
	plt.pause(0.01)
	plt.draw()























