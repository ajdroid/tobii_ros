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
import os
from subprocess import call
from publisher import *
import traceback
import cv2
import imagezmq
import numpy as np
from time import sleep
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# setup socket to python3 video streamer

# Helper class implementing an IO daemon thread for imgzmq recv
class VideoStreamSubscriber:

    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        self._stop = False
        self._data_ready = threading.Event()
        self._thread = threading.Thread(target=self._run, args=())
        self._thread.daemon = True
        self._thread.start()

    def receive(self, timeout=30.0):
        flag = self._data_ready.wait(timeout=timeout)
        if not flag:
            raise TimeoutError(
                "Timeout while reading from subscriber tcp://{}:{}".format(self.hostname, self.port))
        self._data_ready.clear()
        return self._data

    def _run(self):
        receiver = imagezmq.ImageHub("tcp://{}:{}".format(self.hostname, self.port), REQ_REP=False)
        while not self._stop:
            self._data = receiver.recv_jpg()
            # self._data = receiver.recv_image()
            self._data_ready.set()
        receiver.close()

    def close(self):
        self._stop = True

# Receive from broadcast
# There are 2 hostname styles; comment out the one you don't need
hostname = "127.0.0.1"  # Use to receive from localhost
# hostname = "192.168.86.38"  # Use to receive from other computer


if __name__ == '__main__':
    try:
        # parser = argparse.ArgumentParser()
        # parser.add_argument('--gp', action="store_true", help="Option to publish gaze position (2D) data")
        # args = parser.parse_args(rospy.myargv()[1:])

        '''
        Initiate the Video Stream Subscription over Image ZMQ
        '''
        imgzmq_port = 5555
        receiver = VideoStreamSubscriber(hostname, imgzmq_port)

        '''
        Create publishers
        '''
        # Default publish the 3D gaze position data
        vidpub = rospy.Publisher("tobii_video", Image, queue_size=10)
        vid_timestamp_pub = rospy.Publisher("tobii_pts", String, queue_size=10)
        bridge = CvBridge()

        rospy.init_node('tobii_image_sender', anonymous=True)

        while not rospy.is_shutdown():
            # get from py3
            sent_msg_string, frame = receiver.receive()
            image = cv2.imdecode(np.frombuffer(frame, dtype='uint8'), -1)
            print(image.shape, sent_msg_string)

            # publish to ROS
            vidpub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
            vid_timestamp_pub.publish(sent_msg_string)

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        sys.exit(0)
