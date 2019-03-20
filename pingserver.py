#!usr/bin python3 

""" Request distance measurements from a Blue Robotics Ping1D device over udp (PingProxy)
    Send results to autopilot via mavproxy over udp for use as mavlink rangefinder
    Don't request if we are already getting data from device (ex. there is another client
    (pingviewer gui) making requests to the proxy)
"""

#incl socket lib
import socket

# Include bluerobotics-ping
from brping import pingmessage

# Set address
#ADDRESS = ("192.168.2.2", 9090)
ADDRESS = ("192.168.2.2", 5600)
 
# Set socket configuration
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.settimeout(1.0)

# Create pingmessage parser
PARSER = pingmessage.PingParser()


# Send a request for new messages from id
def request(id: int):
    msg = pingmessage.PingMessage()
    msg.request_id = id
    msg.pack_msg_data()
    SOCK.sendto(msg.msg_data, ADDRESS)

# Parse data to create new ping messages
def parse(data):
    for b in bytearray(data):
        if PARSER.parse_byte(b) == PARSER.NEW_MESSAGE:
            return PARSER.rx_msg
    return{}

while True:
    request(pingmessage.PING1D_DISTANCE_SIMPLE)

    try:
        data, addr = SOCK.recvfrom(1024)
        print(parse(data))
    
    except socket.timeout:
        print('REQUEST TIMED OUT')

