# -*- coding: utf-8 -*-

import sys
import json
import rospy
import time
from gaze.msg import Gaze, Pts, Syncsig#Gp3, Gp, Ac, Gy, Gaze

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

'''
A dictionary mapping strings to imported objects. This is used
when creating a publisher. We need to pass the import (not as a function call??).
'''
# imports = {
#     "gp3" : Gp3,
#     "gp" : Gp,
#     "ac" : Ac,
#     "gy" : Gy,
#     "gaze" : Gaze
# }


gaze_topics = ["gp3", "gp", "ac", "gy", "gaze"]
sync_topics = ["pts", "sig"]

topics = gaze_topics + sync_topics

# '''
# A dictionary mapping strings to imported object's __init__.
# This is used when we create a message to publish.
# I'm not sure this is totally necessary, but my knowledge of python
# isn't advanced enough to not use this.
# '''
# msg_types = {
#     "gp3" : Gp3(),
#     "gp" : Gp(),
#     "ac" : Ac(),
#     "gy" : Gy(),
#     "gaze" : Gaze()
# }

class InvalidTopicException(Exception):
    pass

'''
A pseudo-factory to create each publisher on the same node.
'''
class Publisher_Factory:
    def __init__(self):
        rospy.init_node("gazePublisher", anonymous=True)

    def make_Publisher(self, topic):
        return Publisher(topic)

'''
A publisher class. Used to create a publisher for each topic we want to publish.
topic : the topic we're publishing to. Each packet from the glasses get's their own topic.
We create a specific publisher for each topic.

@Method publish : Takes the raw JSON data from the glasses and
creates a msg object based on the attributes.
'''
class Publisher:
    _topic = None
    _data = None
    _pub = None
    _rate = None

    '''
    Init to create a publisher on a specific topic following the "tobii + [TOPIC]"
    convention. This is so that ROS packages can choose which topic to consume.
    We also defined it with the specific message packet that corresponds to the topic.
    '''
    def __init__(self, topic, ):
        self._topic = topic
        if not topic in topics:
            raise InvalidTopicException("Undefined topic.")
        elif topic in gaze_topics:
            self.publish = self.gaze_publish
            self._pub = rospy.Publisher("tobiigaze" + topic, Gaze, queue_size=10, tcp_nodelay=True)
        elif topic is "sig":
            self.publish = self.syncsig_publish
            self._pub = rospy.Publisher("tobii" + topic, Syncsig, queue_size=10, tcp_nodelay=True)
        elif topic is "pts":
            self.publish = self.pts_publish
            self._pub = rospy.Publisher("tobii" + topic, Pts, queue_size=10, tcp_nodelay=True)
        print("Creating publisher for gaze" + topic + " topic")

        self._rate = rospy.Rate(10)

    '''
    Methods to publish data to a specific publisher, expects data to be encoded JSON.
    After decoding, we match the JSON object's properties to the message properties.
    If the 's' property is 1, the packet is invalid (for multiple possible reasons)
    '''


    def gaze_publish(self, data):
        self._data = json.loads(data.decode("utf-8", "replace"))

        if self._data["s"] == 0:
            msg = Gaze()
            for k, v in self._data.iteritems():
                if k == "ts":
                    msg.ts = v
                if k in gaze_topics:
                    msg.vector = v
            print(msg)
            rospy.loginfo(msg)
            self._pub.publish(msg)
            self._rate.sleep()
        elif self._data["s"] == 1:
            print(self._data)
        return

    def pts_publish(self, data):
        self._data = json.loads(data.decode("utf-8", "replace"))

        if self._data["s"] == 1 or self._data["s"] == 0:
            msg = Pts()
            msg.ts = self._data["ts"]
            msg.pts = self._data["pts"]
            msg.pv = self._data["pv"]

            print(msg)
            rospy.loginfo(msg)
            self._pub.publish(msg)
            self._rate.sleep()
        return

    def syncsig_publish(self, data):
        self._data = json.loads(data.decode("utf-8", "replace"))

        if self._data["s"] == 1 or self._data["s"] == 0:
            msg = Pts()
            msg.ts = self._data["ts"]
            msg.dir = self._data["dir"]
            msg.sig = self._data["sig"]

            print(msg)
            rospy.loginfo(msg)
            self._pub.publish(msg)
            self._rate.sleep()
        return

