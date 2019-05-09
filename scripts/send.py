import socket

UDP_IP = "192.168.1.4"
UDP_PORT = 6000
#MESSAGE = "I -43.220121 144.23233 " + str(180)
MESSAGE = "G -43.220 144.232 45.832"
#MESSAGE = "W -43.220 144.232"
print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
