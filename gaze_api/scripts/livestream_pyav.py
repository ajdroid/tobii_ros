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
import cv2
import av
import json


timeout = 1.0
running = True
# Global gaze position to indicate the point of gaze on the video.
gp = [0, 0]
# This is necessary because the async-ness that the socket has.
ts = 0

GLASSES_IP = "192.168.0.102"  # IPv4 address
PORT = 49152


# Keep-alive message content used to request live data and live video streams
KA_DATA_MSG = "{\"type\": \"live.data.unicast\", \"key\": \"some_GUID\", \"op\": \"start\"}"
KA_VIDEO_MSG = "{\"type\": \"live.video.unicast\", \"key\": \"some_other_GUID\", \"op\": \"start\"}"


# Create UDP socket
def mksock(peer):
    iptype = socket.AF_INET
    if ':' in peer[0]:
        iptype = socket.AF_INET6
    return socket.socket(iptype, socket.SOCK_DGRAM)


# Callback function
def send_keepalive_msg(socket, msg, peer):
    while running:
        socket.sendto(bytes(msg, 'UTF-8'), peer)
        time.sleep(timeout)

def signal_handler(signal, frame):
    stop_sending_msg()
    sys.exit(0)

def stop_sending_msg():
    global running
    running = False
    cv2.destroyAllWindows()
    print("stopped sending message")

def receive_in_background(data_socket):
    '''
    Method to recieve the gaze position data in the background and update the gp variable.
    @param data_socket is the socket to receive data from.
    '''
    global running, gp, ts
    while running:
        data, address = data_socket.recvfrom(1024)
        #print (data)
        if data:
            dec_data = data.decode("utf-8", "replace")
            if 'gp3' in dec_data:
                pass
            elif 'gp' in dec_data:
                ob = json.loads(data.decode("utf-8", "replace"))
                gp = ob['gp']
                ts = ob['ts']
                print(ob['ts'])
            elif 'dir' in dec_data:
                ob = json.loads(data.decode("utf-8", "replace"))
                sync_ts = ob['ts']
                sync_sig = ob['sig'] # 1 if active, 0 if not


if __name__ == "__main__":
    global gp
    signal.signal(signal.SIGINT, signal_handler)
    peer = (GLASSES_IP, PORT)

    # Create socket which will send a keep alive message for the live data stream
    data_socket = mksock(peer)
    td = threading.Timer(0, send_keepalive_msg, [data_socket, KA_DATA_MSG, peer])
    td.start()

    # Create socket which will send a keep alive message for the live video stream
    video_socket = mksock(peer)
    tv = threading.Timer(0, send_keepalive_msg, [video_socket, KA_VIDEO_MSG, peer])
    tv.start()

    # Create a thread to receive data in the background
    bd = threading.Thread(target=receive_in_background, args=(data_socket, ))
    bd.daemon = True
    bd.start()

    # Create a video capture device from the rtsp server running on the
    # recording unit. Only works with IPv4.
    # cap = cv2.VideoCapture("rtsp://%s:8554/live/scene" % GLASSES_IP)
    # # Reduces the amount of h264 decoding errors (only in OpenCV3+)
    # cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    # font = cv2.FONT_HERSHEY_DUPLEX

    container = av.open("rtsp://%s:8554/live/scene" % GLASSES_IP, options={'rtsp_transport': 'tcp'})
    stream = container.streams.video[0]

    for frame in container.decode(stream):
        # Read live data
        frame_cv = frame.to_ndarray(format='bgr24')

        # if data_gp['ts'] > 0 and data_pts['ts'] > 0:
        #     offset = data_gp['ts'] / 1000.0 - data_pts['ts'] / 1000.0  # in milliseconds
        #     print('Frame_pts = %f' % float(frame.pts))
        #     print('Frame_time = %f' % float(frame.time))
        #     print('Data_pts = %f' % float(data_pts['pts']))
        #     print('Offset = %f' % float(offset))

        if not frame_cv:
            print("no return frame")

        # Place an 'O' on the video where the gaze position is currently at.
        # Video is at a 1920x1080 resolution. The point is a normalized vector in 2D
        # with the origin in the upper-left corner, just like opencv frames
        # We can just multiply the point by the dimensions to get the location
        height, width = frame_cv.shape[:2]
        cv2.circle(frame_cv, (int(gp['gp'][0] * width), int(gp['gp'][1] * height)), 20, (0, 0, 255), 6)

        cv2.imshow('video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            stop_sending_msg()
