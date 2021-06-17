# Object detection and avoidance with Arducopter using a ZED Stereo Cam 
## Project Introduction
In this project we use a ZED Stereo camera for 3D distance measurements and object detection based on Yolo for obstacle avoidance. The point cloud is processed on a Jetson TX2 and the resulting 3D distance information and data on detected obstacles is transferred to a Pixhawk flightcontroller. The Behaviour of the drone in case of detected obstacles ahead is handled in Arducopter's object avoidance functionality. For a detailed overview, see here: https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html and here: https://ardupilot.org/copter/docs/common-oa-bendyruler.html

## Prerequisites 
This project runs on an Nvidia Jetson TX2 connected to Arducopter 4.0.7. The drone is built on a Tarot frame.

![The Tarot 650 based copter used for this project](https://github.com/mtbsteve/redtail/blob/master/tools/images/image4.jpeg)

### Hardware requirements
- Jetson TX2 with Jetpack 4.4 or higher
- Pixhawk Cube with Arducopter 4.0.7 or higher installed
- a drone equipped with a ZED Stereo camera, example setup see here: https://github.com/mtbsteve/redtail/wiki
- The ZED is mounted on a gimbal in order to keep the camera always leveled (no software pitch compensation used)

### Software requirements
- ZED SDK 3.4.2 or higher including the ZED Python wrapper
- ROS Melodic full installation including rospy 
- APSync needs to be installed on the TX2 for the mavlink and GCS communication see: https://github.com/mtbsteve/companion/tree/master/Nvidia_JTX2_JP44/Ubuntu
- Python3 along with numpy, pymavlink, pyserial, apscheduler
- Darknet/yolov4 https://github.com/AlexeyAB/darknet With the yolov4-tiny dataset, the TX2 is capable of 12-15FPS processing speed which is sufficient for visualization in flight and object detection.
- Note that you need to install cvbridge and to compile it for Python3!
- For video streaming of the different image nodes you need to install the ROS to RTSP node https://github.com/CircusMonkey/ros_rtsp

## Installation

Please see the wiki (https://github.com/mtbsteve/pegasus/wiki) for installation details.

## Next steps

- support of Arducopter 4.1 including the OBSTACLE_DISTANCE_3D message
- Implement detected yolo objects as obstacles and send them to Ardupilot
