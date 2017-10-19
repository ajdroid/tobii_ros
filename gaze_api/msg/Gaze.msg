#ROS module for Tobii Pro Glasses
#Copyright 2017 Carnegie Mellon University. All Rights Reserved.
#NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT INFRINGEMENT.
#Released under a BSD-style license, please see license.txt or contact permission@sei.cmu.edu for full terms.
#[DISTRIBUTION STATEMENT A] This material has been approved for public release and unlimited distribution.  Please see Copyright notice for non-US Government use and distribution.
#This Software includes and/or makes use of the following Third-Party Software subject to its own license.  By using this Software you agree:
#1. Tobii Pro Glasses 2 API. Copyright 2017 Tobii AB (publ).
#A.	You may not copy (except for backup purposes), modify, adapt, decompile, reverse engineer, disassemble, or create derivative works of the files (for example dynamic-link library files, commonly referred to as DLL-files), object code or other components of the Tobii Pro Glasses 2 API (the ?Software Components?) that are intended to be re-used in Applications for end users, and any Updates, modifications and/or patches or hot fixes thereto that Tobii may make generally available from time to time or any part thereof. 
#B.	You may not redistribute or combine/bundle any part of the Software Components with other software, or distribute any software or device incorporating part of the Software Components. 
#C.	You agree that Tobii AB (i) owns all legal right, title and interest in and to the Software Components, including any related intellectual property rights; and (ii) reserves all rights not expressly granted. 
#D.  	You agree that You have no right to use any of Tobii's trade names, trademarks, service marks, logos, domain names, or other distinctive brand features. 
#5) 	You agree that you will not remove, obscure, or alter any proprietary rights notices (including copyright and trademark notices) that may be affixed to or contained within the Software Components.
#DM17-0454



# Every gaze message contains a timestamp sent by the recording unit
float64 ts

# Each message contains some sort of vector (2D or 3D)
# topics will contain a vector that corresponds to that information
# gp := Gaze position
#   ts : time stamp
#   gp : The position on the scene camera image where the gaze will be projected
#         The top left corner is (0, 0), bottom right corner is (1, 1)

# gp3 := Gaze position 3d
#   ts : time stamp
#   gp3 : Gaze Position 3d in the 3D position, in mm, relative to the scene camera
#         where the gaze is focused.

# gy := MEMS gyroscope info
#   ts : time stamp
#   gy : The gyroscope data indicates the rotation of the glasses. The gyroscope data
#       has the unit degrees per second.

# ac := MEMS accelerometer info
#   ts : time stamp
#   ac : The accelerometer data indicates the rotation of the glasses. The accelerometer data
#        has the unit meter per second squared. When the glasses are stationary, the value of ac
#        property will be approximately [0, -9,82, 0].

float32[] vector
