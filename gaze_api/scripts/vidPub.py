#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
import zmq

# setup socket to python3 video streamer
# reading images and image associated pts from tobii


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()

        parser.add_argument('--gp', action="store_true", help="Option to publish gaze position (2D) data")

        args = parser.parse_args(rospy.myargv()[1:])

        '''
        Start initiating socket connections. Place both data and video socket on background process
        that runs every second.
        '''
        context = zmq.Context()
        # Socket to send messages on
        sender = context.socket(zmq.PUSH)
        sender.bind("tcp://*:5557")

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
        sigpub = pub_factory.make_Publisher(SYNC_SIG)
        ptspub = pub_factory.make_Publisher(VIDEO_PTS)
        gppub = pub_factory.make_Publisher(GAZE_POSITION_2D)
        gypub = None
        acpub = None

        # Create a publisher for optional topics
        if args.gy:
            gypub = pub_factory.make_Publisher(GYROSCOPE)
        if args.ac:
            acpub = pub_factory.make_Publisher(ACCELEROMETER)

        '''
        On each data receive, check which publisher is responsible for
        publishing and then send it out.
        '''
        while not rospy.is_shutdown():
            data, address = data_socket.recvfrom(1024)

            dec_data = data.decode("utf-8", "replace")
            if VIDEO_PTS in dec_data and acpub:
                ptspub.publish(data)
            if SYNC_SIG in dec_data and acpub:
                sigpub.publish(data)
            if ACCELEROMETER in dec_data and acpub:
                acpub.publish(data)
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
