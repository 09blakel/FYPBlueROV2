#!/usr/bin/env python3

""" Request distance measurements from a Blue Robotics Ping1D device over udp (PingProxy)
    Send results to autopilot via mavproxy over udp for use as mavlink rangefinder
    Don't request if we are already getting data from device (ex. there is another client
    (pingviewer gui) making requests to the proxy)
"""

import argparse
import errno
import socket
import time

from pymavlink import mavutil
from brping import pingmessage

PARSER = argparse.ArgumentParser(description="Ping1D to mavlink bridge.")
PARSER.add_argument('--ping',
                    action="store",
                    type=str,
                    default="0.0.0.0:9090",
                    help="Ping device udp address and port. ex \"0.0.0.0:9090\""
                    )
PARSER.add_argument('--mavlink',
                    action="store",
                    type=str,
                    default="0.0.0.0:9000",
                    help="Mavlink udp address and port. ex \"0.0.0.0:9000\""
                    )
PARSER.add_argument('--min-confidence',
                    action="store",
                    type=int,
                    default=50,
                    help="Minimum acceptable confidence percentage for depth measurements.\""
                    )
ARGS = PARSER.parse_args()

def main():
    """ Main function
    """

    ## The time that this script was started
    tboot = time.time()

    ## Parser to decode incoming PingMessage
    ping_parser = pingmessage.PingParser()

    ## Messages that have the current distance measurement in the payload
    distance_messages = [
        pingmessage.PING1D_DISTANCE,
        pingmessage.PING1D_DISTANCE_SIMPLE,
        pingmessage.PING1D_PROFILE
        ]

    ## The minimum interval time for distance updates to the autopilot
    ping_interval_ms = 0.1

    ## The last time a distance measurement was received
    last_distance_measurement_time = 0

    ## The last time a distance measurement was requested
    last_ping_request_time = 0

    pingargs = ARGS.ping.split(':')
    pingserver = (pingargs[0], int(pingargs[1]))

    autopilot_io = mavutil.mavlink_connection("udpout:" + ARGS.mavlink,
                                              source_system=1,
                                              source_component=192
                                              )

    ping1d_io = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ping1d_io.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ping1d_io.setblocking(False)

    ## Send a request for distance_simple message to ping device
    def send_ping1d_request():
        data = pingmessage.PingMessage()
        data.request_id = pingmessage.PING1D_DISTANCE_SIMPLE
        data.src_device_id = 0
        data.pack_msg_data()
        ping1d_io.sendto(data.msg_data, pingserver)

    # some extra information for the DISTANCE_SENSOR mavlink message fields
    min_distance = 20
    max_distance = 5000
    sensor_type = 2
    orientation = 25
    covarience = 0

    ## Send distance_sensor message to autopilot
    def send_distance_data(distance, deviceid, confidence):
        print("sending distance %d confidence %d" % (distance, confidence))
        if confidence < ARGS.min_confidence:
            distance = 0

        autopilot_io.mav.distance_sensor_send(
            (time.time() - tboot) * 1000, # time_boot_ms
            min_distance, # min_distance
            max_distance, # max_distance
            distance/10, # distance
            sensor_type, # type
            deviceid, # device id
            orientation,
            covarience)

    while True:
        time.sleep(0.001)
        tnow = time.time()

        # Declare our variables
        last_distance_measurement_time = 0
        distance = 0
        deviceid = 0
        confidence = 0


        # request data from ping device
        if tnow > last_distance_measurement_time + ping_interval_ms:
            if tnow > last_ping_request_time + ping_interval_ms:
                last_ping_request_time = time.time()
                send_ping1d_request()

        # read data in from ping device
        try:
            data, _ = ping1d_io.recvfrom(4096)
            print(data)
        except socket.error as exception:
            # check if it's waiting for data
            if exception.errno != errno.EAGAIN:
                raise exception


        # decode data from ping device, forward to autopilot
        for byte in data:
            if ping_parser.parseByte(byte) == pingmessage.PingParser.NEW_MESSAGE:
                if ping_parser.rx_msg.message_id in distance_messages:
                    last_distance_measurement_time = time.time()
                    distance = ping_parser.rx_msg.distance
                    print(distance)
                    deviceid = ping_parser.rx_msg.src_device_id
                    confidence = ping_parser.rx_msg.confidence
                    send_distance_data(distance, deviceid, confidence)

        # My code goes here
        #if distance > max_distance :
            #send(kil)/ send disarm to BlueROV 2
if distance <= 3:
    mavutil.set_dialect("ardupilotmega")
    autopilot = mavlink.mavlink_connection('udpin:192.168.2.1:14550')

    msg = None

    #wait for autopilot connection
    while msg is None:
            msg = autopilot.recv_msg()


    print msg

    autopilot.mav.heartbeat_send(
    6, #type
    8, #autopilot
    192, #base_mode
    0, #custom_mode
    4, #system_status
    3 #mavlink_version
    )


    autopilot.mav.command_long_send(
    1, #autopilot sys id
    1, #autopilot component id
    400, #command id (arm or disarm)
    0, #confirmation
    0, #disarm!
    0,0,0,0,0,0, # unused parameters for the disarm command
    )

    time.sleep(2)


if __name__ == '__main__':
    main()
