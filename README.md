# Object detection and avoidance with Arducopter using a ZED Stereo Cam 
## Project Introduction
In this project we use a ZED Stereo camera for 3D distance measurements and object detection based on Yolo for obstacle avoidance. The point cloud is processed on a Jetson TX2 and the resulting 3D distance information and data on detected obstacles is transferred to a Pixhawk flightcontroller. The Behaviour of the drone in case of detected obstacles ahead is handled in Arducopter's object avoidance functionality. For a detailed overview, see here: https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html and here: https://ardupilot.org/copter/docs/common-oa-bendyruler.html

## Prerequisites
This project runs on an Nvidia Jetson TX2 connected to Arducopter 4.0.7

### Hardware requirements
- Jetson TX2 with Jetpack 4.4 or higher
- Pixhawk Cube with Arducopter 4.0.7 or higher installed
- a drone equipped with a ZED Stereo camera, example setup see here: https://github.com/mtbsteve/redtail/wiki
- The ZED is mounted on a gimbal in order to keep the camera always leveled (no software pitch compensation used)

The 3D parts needed to build a simple 1-axis [gimbal] (https://github.com/mtbsteve/redtail/tree/master/tools/platforms/ZED_Gimbal) can be found under the link. One axis is sufficient to level the camera around the nick-axis. We do not need the roll or yaw axis stabilization since we won't create cinema-grade footage anyway and we do want to keep the weight as low as possible.

![The ZED 1 - axis gimbal](https://github.com/mtbsteve/redtail/blob/master/tools/images/image0.jpeg)

### Software requirements
- ZED SDK 3.4.2 or higher including the ZED Python wrapper
- ROS Melodic full installation including rospy 
- APSync needs to be installed on the TX2 for the mavlink and GCS communication see: https://github.com/mtbsteve/companion/tree/master/Nvidia_JTX2_JP44/Ubuntu
- Python3 along with numpy, pymavlink, pyserial, apscheduler
- Darknet/yolov4 https://github.com/AlexeyAB/darknet With the yolov4-tiny dataset, the TX2 is capable of 12-15FPS processing speed which is sufficient for visualization in flight and object detection.
- Note that you need to install cvbridge and to compile it for Python3!
- For video streaming of the different image nodes you need to install the ROS to RTSP node https://github.com/CircusMonkey/ros_rtsp

## Installation
In order to communicate to mavlink through mavros, make sure you have opened a UDP port in your mavlink-router. Edit `~/start_mavlink-router/mavlink-router.conf` and add the following lines at the end:
```
[UdpEndpoint to_ros]
Mode = Normal
Address = 127.0.0.1
Port = 14855
```

Next, create a catkin workspace and navigate to the src directory:
```
cd ~/cd $CATKIN_WS/src
mkdir zed_yolo && cd zed_yolo
git clone --recursive https://github.com/mtbsteve/pegasus.git
cd ..
catkin build zed_yolo -DCMAKE_BUILD_TYPE=Release
```
This project provides two ROS nodes, `zed_yolo zed_yolo_node.py` and `zed_yolo zed_ros_to_mavlink.py`. The first node generates a point cloud and the left camera image from by the ZED camaera stereo images. It runs the darknet-yolo detection and creates three ROS topics:
```
/darknet_ros/color_image the left camera image with the distance to the next obstacle information in the the center of the image
/darknet_ros/detection_image  the yolo view with the detected objects marked with a bounding box and average distance information from the camera
/darknet_ros/distance_array  the information needed by the OBSTACLE_DISTANCE function of Mavlink
```
The image topics can be streamed with the ROS to RTSP node to any RTSP video stream capable ground station, eg QGroundControl, Mission Planner or Solex.

The `zed_yolo zed_ros_to_mavlink.py` node reads from the /darknet_ros/distance_array topic and passes the information on to the Flight Controller.

To start:
```
# start the ROS to RTSP streaming node
roslaunch ros_rtsp rtsp_streams.launch
# launch the YOLO detection and image node
roscore &
rosrun zed_yolo zed_yolo_node.py
# launch the Mavlink communication node
rosrun zed_yolo zed_ros_to_mavlink.py
```

