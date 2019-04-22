import os, subprocess
from PyQt5.QtWidgets import *

# Setup application
app = QApplication([])
window = QWidget()
#window.resize(720, 480)
layout = QVBoxLayout()





# System variables
#width = 480
#height = 480
#framerate = 30
filename = "stereo_image"
port = 5000
#devIndex = [0, 1]








# Store GStreamer pipelines for both direct camera feed and for UDP streaming
#gst_direct_1 = ("gst-launch-1.0 ksvideosrc device-index={} ! video/x-raw, width={}, height={}, framerate={}/1".format(devIndex[0], width, height, framerate)
#	+ " ! compositor name=comp sink_1::xpos={} ! queue ! videoconvert ! jpegenc ! ".format(width))
#gst_direct_2 = (" ksvideosrc device-index={} ! video/x-raw, width={}, height={}, framerate={}/1 ! comp.".format(devIndex[1], width, height, framerate))

def gst_save():
	return "videoconvert ! jpegenc ! multifilesink location={}.jpg".format(filename)

def gst_view():
	return "autovideosink"

def gst_udp():
	return ("gst-launch-1.0 udpsrc port={} ! queue ! application/x-rtp, encoding-name=JPEG, payload=26 ! rtpjpegdepay ! jpegdec ! ".format(port))




def UDP_Save ():
	while True:
		os.system(gst_udp() + gst_save())
		print("GStreamer closed due to an error. Relaunching stream...")

def UDP_View ():
	while True:
		os.system(gst_udp() + gst_view())
		print("GStreamer closed due to an error. Relaunching stream...")





'''
# Every time GStreamer stops running, run it again
while True:
	if isUDP:
		if isSave:
			os.system(gst_udp() + gst_save())
		else:
			os.system(gst_udp() + gst_view)
	else:
		if isSave:
			os.system(gst_direct_1 + gst_save() + gst_direct_2)
		else:
			os.system(gst_direct_1 + gst_view + gst_direct_2)

	print("GStreamer closed due to an error. Relaunching stream...")
'''


cb_camera = QComboBox()
label_port = QLabel('Port:')
txt_port = QLineEdit(str(port))
label_filename = QLabel('Filename:')
txt_filename = QLineEdit(str(filename))

cb_camera.addItems(['Stereo Camera', 'Arm Camera', 'Black Foscam'])
def cb_camera_change(i):
	global port, filename

	if i == 0: # Stereo Camera
		port = 5000
		filename = 'stereo_image'
		txt_port.setText(str(port))
		txt_filename.setText(filename)

	elif i == 1: # Arm Camera
		port = 5001
		filename = 'arm_image'
		txt_port.setText(str(port))
		txt_filename.setText(filename)

	elif i == 2: # Arm Camera
		port = 5002
		filename = 'foscam_image'
		txt_port.setText(str(port))
		txt_filename.setText(filename)

cb_camera.currentIndexChanged.connect(cb_camera_change)
layout.addWidget(cb_camera)


layout.addWidget(label_port)
layout.addWidget(txt_port)


layout.addWidget(label_filename)
layout.addWidget(txt_filename)






# Create View Stream Button
btn_View = QPushButton('View Stream')
def on_btn_View():
	global port

	# Update variables
	port = txt_port.text()

	window.hide()

	UDP_View()
btn_View.clicked.connect(on_btn_View)
layout.addWidget(btn_View)



# Create save stream button
btn_Save = QPushButton('Save Stream')
def on_btn_Save():
	global port, filename

	# Update variables
	port = txt_port.text()
	filename = txt_filename.text()

	window.hide()

	UDP_Save()
btn_Save.clicked.connect(on_btn_Save)
layout.addWidget(btn_Save)



# Run window
window.setLayout(layout)
window.show()

app.exec_()