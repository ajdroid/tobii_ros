# Tobii Glasses 2 on ROS
This is a ROS package originally developed by the [Emerging Technology Center](http://www.sei.cmu.edu/about/organization/etc/) (ETC) at [Carnegie Mellon University](http://www.cmu.edu/)'s [Software Engineering Institute](http://www.sei.cmu.edu) for broadcasting live data from the Tobii Glasses 2.

Note, this package is intended for ROS Indigo using the Catkin build process.

For more information about the Tobii Glasses 2 API, please consult the published API found [here](https://www.tobiipro.com/product-listing/tobii-pro-glasses-2-sdk/).

This application is powered by Tobii.

<img src="https://www.tobiipro.com/imagevault/publishedmedia/9f5pqmy21ou5wpmv3s9l/TobiiPro-Colorlogo-TransparentBackground-818x300.png?download=1" alt="Tobii Logo" style="width: 200px;"/>

## Dependencies
Besides some default Python packages, this ROS node requires opencv3 or greater to be installed. Follow this [guide](http://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/). Note, it's easiest to install Python 3.5 and link opencv to that since ROS uses Python 2.7 and **will** override your environment.

# ROS API
## gazePub
The best way to start the gazePub node is to first make sure `roscore` is running on a separate terminal. Then, `rosrun gaze gazePub.py` in a new terminal.

### Topics
Notice, all topics follow the theme of **gaze** + **[TOPIC]** where topic is the flag passed to the publishing node (i.e. **gy**, **ac**, etc).  
* `gazegp3`  
    * gp3 : Gaze Position 3d in the 3D position, in mm, relative to the scene camera where the gaze is focused.  
* `gazegp`  
    * gp : The position on the scene camera image where the gaze will be projected. The top left corner is (0, 0), bottom right corner is (1, 1)  
* `gazegy`  
    * gy : The gyroscope data indicates the rotation of the glasses. The gyroscope data has the unit degrees per second.  
* `gazeac`  
    * ac : The accelerometer data indicates the rotation of the glasses. The accelerometer data has the unit meter per second squared. When the glasses are stationary, the value of ac property will be approximately [0, -9,82, 0].  

### Message type  
`gaze.msg/Gaze`  
* `float64 ts`  
    * A monotonic time stamp given by the recording unit  
* `float32[] vector`  
    * A vector of information depending on the topic. i.e. `gazegp3` publishes a 3D vector containing the Gaze Position to the `gazegp3` topic.  

## livestream.py
This is the module which streams live video from the glasses and adds the gaze overlay, as reported by the recording unit. This is the file which requires opencv3 or greater. As noted above, it's easiest to use Python3.5 to run this module due to conflicts with ROS.
`python3 livestream.py`

### Running Tests
First, from the source of the workspace, run:  
`catkin_make`  
`source devel/setup.bash`  
`rostest gaze gaze.launch`
  * For running all three tests. Two tests require the recording unit to be turned on and connected.
  * Or run `rostest gaze [TEST_NAME]` or an individual test.

## Authors
Eric Schmar
* etschmar [ AT ] sei [ DOT ] cmu [ DOT ] edu
* personal: ericschmar [ AT ] gmail [ DOT ] com  

Patrick Dwyer
* pmdwyer [ AT ] sei [ DOT ] cmu [ DOT ] edu  

Ritwik Gupta
* rgupta [ AT ] sei [ DOT ] cmu [ DOT ] edu  

Jonathan Chu
* jchu [ AT ] sei [ DOT ] cmu [ DOT ] edu

## License & Copyright Information

ROS module for Tobii Pro Glasses

Copyright 2017 Carnegie Mellon University. All Rights Reserved.

BSD

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

ROS module for Tobii Pro Glasses includes and/or can make use of certain third party software ("Third Party Software"). The Third Party Software that is used by ROS module for Tobii Pro Glasses is dependent upon your system configuration, but typically includes the software identified below. By using ROS module for Tobii Pro Glasses, You agree to comply with any and all relevant Third Party Software terms and conditions contained in any such Third Party Software or separate license file distributed with such Third Party Software. The parties who own the Third Party Software ("Third Party Licensors") are intended third party beneficiaries to this License with respect to the terms applicable to their Third Party Software. Third Party Software licenses only apply to the Third Party Software and not any other portion of ROS module for Tobii Pro Glasses or ROS module for Tobii Pro Glasses as a whole.

This material is based upon work funded and supported by the Department of Defense under Contract No. FA8702-15-D-0002 with Carnegie Mellon University for the operation of the Software Engineering Institute, a federally funded research and development center.

The view, opinions, and/or findings contained in this material are those of the author(s) and should not be construed as an official Government position, policy, or decision, unless designated by other documentation.

NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT INFRINGEMENT.

[DISTRIBUTION STATEMENT A] This material has been approved for public release and unlimited distribution.  Please see Copyright notice for non-US Government use and distribution.

This Software includes and/or makes use of the following Third-Party Software subject to its own license.  By using this Software you agree:

1. Tobii Pro Glasses 2 API. Copyright 2017 Tobii AB (publ).

A.	You may not copy (except for backup purposes), modify, adapt, decompile, reverse engineer, disassemble, or create derivative works of the files (for example dynamic-link library files, commonly referred to as DLL-files), object code or other components of the Tobii Pro Glasses 2 API (the �Software Components�) that are intended to be re-used in Applications for end users, and any Updates, modifications and/or patches or hot fixes thereto that Tobii may make generally available from time to time or any part thereof.

B.	You may not redistribute or combine/bundle any part of the Software Components with other software, or distribute any software or device incorporating part of the Software Components.

C.	You agree that Tobii AB (i) owns all legal right, title and interest in and to the Software Components, including any related intellectual property rights; and (ii) reserves all rights not expressly granted.

D.  	You agree that You have no right to use any of Tobii's trade names, trademarks, service marks, logos, domain names, or other distinctive brand features.

5) 	You agree that you will not remove, obscure, or alter any proprietary rights notices (including copyright and trademark notices) that may be affixed to or contained within the Software Components.

DM17-0454

