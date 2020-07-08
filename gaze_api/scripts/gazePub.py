#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
ROS module for Tobii Pro Glasses
Copyright 2017 Carnegie Mellon University. All Rights Reserved.
NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT INFRINGEMENT.
Released under a BSD-style license, please see license.txt or contact permission@sei.cmu.edu for full terms.
[DISTRIBUTION STATEMENT A] This material has been approved for public release and unlimited distribution.  Please see Copyright notice for non-US Government use and distribution.
This Software includes and/or makes use of the following Third-Party Software subject to its own license.  By using this Software you agree:
1. Tobii Pro Glasses 2 API. Copyright 2017 Tobii AB (publ).
A.	You may not copy (except for backup purposes), modify, adapt, decompile, reverse engineer, disassemble, or create derivative works of the files (for example dynamic-link library files, commonly referred to as DLL-files), object code or other components of the Tobii Pro Glasses 2 API (the �Software Components�) that are intended to be re-used in Applications for end users, and any Updates, modifications and/or patches or hot fixes thereto that Tobii may make generally available from time to time or any part thereof. 
B.	You may not redistribute or combine/bundle any part of the Software Components with other software, or distribute any software or device incorporating part of the Software Components. 
C.	You agree that Tobii AB (i) owns all legal right, title and interest in and to the Software Components, including any related intellectual property rights; and (ii) reserves all rights not expressly granted. 
D.  	You agree that You have no right to use any of Tobii's trade names, trademarks, service marks, logos, domain names, or other distinctive brand features. 
5) 	You agree that you will not remove, obscure, or alter any proprietary rights notices (including copyright and trademark notices) that may be affixed to or contained within the Software Components.
DM17-0454
'''

import time
import socket
import threading
import signal
import sys
import json
import rospy
import argparse
# import tobii
import os
from subprocess import call
from publisher import *

GAZE_POSITION_3D = "gp3"
GAZE_POSITION_2D = "gp"
ACCELEROMETER = "ac"
GYROSCOPE = "gy"

timeout = 1.0
running = True

# GLASSES_IP = "fe80::76fe:48ff:fe30:7a9f"  # IPv6 address scope global 192.168.71.50
GLASSES_IP = "192.168.0.102"#"192.168.71.50"
PORT = 49152


# Keep-alive message content used to request live data and live  streams
KA_DATA_MSG = "{\"type\": \"live.data.unicast\", \"key\": \"some_GUID\", \"op\": \"start\"}"
KA_VIDEO_MSG = "{\"type\": \"live.video.unicast\", \"key\": \"some_other_GUID\", \"op\": \"start\"}"


# Create UDP socket
def mksock(peer):
    iptype = socket.AF_INET
    if ':' in peer[0]:
        iptype = socket.AF_INET6
    return socket.socket(iptype, socket.SOCK_DGRAM)


# Callback function for the threads
def send_keepalive_msg(socket, msg, peer):
    while running:
        socket.sendto(str.encode(msg), peer)
        time.sleep(timeout)

def signal_handler(signal, frame):
    stop_sending_msg()
    sys.exit(0)


def stop_sending_msg():
    global running
    running = False


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()

        parser.add_argument('--gp', action="store_true", help="Option to publish gaze position (2D) data")
        parser.add_argument('--gy', action="store_true", help="Option to publish gyroscope data")
        parser.add_argument('--ac', action="store_true", help="Option to publish accelerometer data")
        parser.add_argument('--lo', action="store_true", help="Testing param to set the IP to localhost")

        args = parser.parse_args(rospy.myargv()[1:])

        '''
        Start initiating socket connections. Place both data and video socket on background process
        that runs every second.
        '''
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        # Allow to connect to local host for testing
        if(args.lo):
            print("Enabling local host connection")
            peer = ("localhost", PORT)
        else:
            peer = (GLASSES_IP, PORT)

        # Create socket which will send a keep alive message for the live data stream
        data_socket = mksock(peer)
        if(args.lo):
            print("binding...")
            data_socket.bind(peer)

        td = threading.Timer(0, send_keepalive_msg, [data_socket, KA_DATA_MSG, peer])
        td.start()

        # Create socket which will send a keep alive message for the live video stream
        # Need both keep alive messages in order to receive any data at all
        video_socket = mksock(peer)
        tv = threading.Timer(0, send_keepalive_msg, [video_socket, KA_VIDEO_MSG, peer])
        tv.start()

        '''
        End thread initiation.
        '''

        '''
        Create a publisher factory which returns a publisher for a
        specific topic
        '''
        pub_factory = Publisher_Factory()
        # Default publish the 3D gaze position data
        gp3pub = pub_factory.make_Publisher(GAZE_POSITION_3D)
        gppub = None
        gypub = None
        acpub = None

        # Create a publisher for each topic parameter chosen
        if(args.gp):
            gppub = pub_factory.make_Publisher(GAZE_POSITION_2D)
        if(args.gy):
            gypub = pub_factory.make_Publisher(GYROSCOPE)
        if(args.ac):
            acpub = pub_factory.make_Publisher(ACCELEROMETER)

        '''
        On each data receive, check which publisher is responsible for
        publishing and then send it out.
        '''
        while not rospy.is_shutdown():
            data, address = data_socket.recvfrom(1024)

            dec_data = data.decode("utf-8", "replace")
            if GAZE_POSITION_3D in dec_data and gp3pub:
                gp3pub.publish(data)
            if GAZE_POSITION_2D in dec_data and gppub:
                gppub.publish(data)
            if GYROSCOPE in dec_data and gypub:
                gypub.publish(data)
            if ACCELEROMETER in dec_data and acpub:
                acpub.publish(data)


    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        stop_sending_msg()
        sys.exit(0)
