#!/usr/bin/env python3


######################################################
##  ROS LidarScan node to MAVLink                   ##
######################################################
# Stephan Schindewolf 04/2021
# pls check that you have the following installed:
#   pip3 install pymavlink
#   pip3 install apscheduler
#   pip3 install pyserial

import sys
sys.path.append("/usr/local/lib/")
sys.path.append("/usr/include/")
sys.path.append("/usr/share/")
sys.path.append('/home/apsync/.local/lib/python3.7/site-packages/')

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
import numpy as np
import math as m
import signal
import sys
import time
import argparse
import threading
import json
from time import sleep
from apscheduler.schedulers.background import BackgroundScheduler
from pymavlink import mavutil
import rospy
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32


# To obtain ip address
import socket

######################################################
##  ArduPilot-related parameters - reconfigurable   ##
######################################################

# Default configurations for connection to the FCU
# Decide which board you are using to connect to the flight controller via mavlink:
# If you are using the J17 connector on the Nvidia TX2 development board
# then set connection_string_default = /dev/ttyTHS2
# if you are using the Auvidea J120 board, then set
# connection_string_default=/dev/ttyTHS1
# if you are using mavlink_router include the IP address for the ROS connection here

connection_string_default = '127.0.0.1:5762'
connection_baudrate_default = 1500000

# Use this to rotate all processed data
camera_facing_angle_degree = 0

# Store device serial numbers of connected camera
device_id = None

# Enable/disable each message/function individually
enable_msg_obstacle_distance = False
enable_3D_msg_obstacle_distance = False
enable_msg_distance_sensor = False
obstacle_distance_msg_hz_default = 15.0
curr_avoid_strategy=""
prev_avoid_strategy=""
# enable only if you are on Arducopter 4.1 or higher
ac_version_41 = True

# lock for thread synchronization
lock = threading.Lock()

mavlink_thread_should_exit = False

# default exit code is failure - a graceful termination with a
# terminate signal is possible.
exit_code = 1

######################################################
##  Global variables                                ##
######################################################

# Camera-related variables
pipe = None
depth_scale = 0
depth_hfov_deg = None
depth_vfov_deg = None

# Data variables
vehicle_pitch_rad = None
data = None
current_time_us = 0
last_obstacle_distance_sent_ms = 0  # value of current_time_us when obstacle_distance last sent

# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# See here: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
distances_array_length = 72
angle_offset = None
increment_f  = None
distances = np.ones((distances_array_length,), dtype=np.uint16) * (2000 + 1)
# Obstacle distances in nine segments for the new OBSTACLE_DISTANCE_3D message
# see here https://github.com/rishabsingh3003/Vision-Obstacle-Avoidance/blob/land_detection_final/Companion_Computer/d4xx_to_mavlink_3D.py
mavlink_obstacle_coordinates = np.ones((9,3), dtype = np.float) * (9999)
debug_enable = 1

######################################################
##  Parsing user' inputs                            ##
######################################################

parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate', type=float,
                    help="Vehicle connection baudrate. If not specified, a default value will be used.")
parser.add_argument('--obstacle_distance_msg_hz', type=float,
                    help="Update frequency for OBSTACLE_DISTANCE message. If not specified, a default value will be used.")
parser.add_argument('--debug_enable',type=float,
                    help="Enable debugging information")
parser.add_argument('--camera_name', type=str,
                    help="Camera name to be connected to. If not specified, any valid camera will be connected to randomly. For eg: type 'D435I' to look for Intel RealSense D435I.")

args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate
obstacle_distance_msg_hz = args.obstacle_distance_msg_hz
debug_enable = args.debug_enable

def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

# Using default values if no specified inputs
if not connection_string:
    connection_string = connection_string_default
    progress("INFO: Using default connection_string %s" % connection_string)
else:
    progress("INFO: Using connection_string %s" % connection_string)

if not connection_baudrate:
    connection_baudrate = connection_baudrate_default
    progress("INFO: Using default connection_baudrate %s" % connection_baudrate)
else:
    progress("INFO: Using connection_baudrate %s" % connection_baudrate)
    
if not obstacle_distance_msg_hz:
    obstacle_distance_msg_hz = obstacle_distance_msg_hz_default
    progress("INFO: Using default obstacle_distance_msg_hz %s" % obstacle_distance_msg_hz)
else:
    progress("INFO: Using obstacle_distance_msg_hz %s" % obstacle_distance_msg_hz)



######################################################
##  Functions - MAVLink                             ##
######################################################

def mavlink_loop(conn, callbacks):
    '''a main routine for a thread; reads data from a mavlink connection,
    calling callbacks based on message type received.
    '''
    interesting_messages = list(callbacks.keys())
    while not mavlink_thread_should_exit:
        # send a heartbeat msg
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                0,
                                0,
                                0)
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None:
            continue
        callbacks[m.get_type()](m)


# https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
def send_obstacle_distance_message():
    global current_time_us, distances, camera_facing_angle_degree
    global last_obstacle_distance_sent_ms
    if (enable_msg_obstacle_distance == True):
        if current_time_us == last_obstacle_distance_sent_ms:
            # no new frame
            progress("no new frame")
            return
        last_obstacle_distance_sent_ms = current_time_us
        print(distances)
        if angle_offset is None or increment_f is None:
            progress("obstacle_distance_params not properly set")
        else:
            #progress("new frame")
            conn.mav.obstacle_distance_send(
                current_time_us,    # us Timestamp (UNIX time or time since system boot)
                0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
                distances,          # distances,    uint16_t[72],   cm
                0,                  # increment,    uint8_t,        deg
                min_depth_cm,	    # min_distance, uint16_t,       cm
                max_depth_cm,       # max_distance, uint16_t,       cm
                increment_f,	    # increment_f,  float,          deg
                angle_offset,       # angle_offset, float,          deg
                12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
            )
            current_time_us = int(round(time.time() * 1000000))


# https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
def send_distance_sensor_message():
    global distances
    # Average out a portion of the centermost part
    if (enable_msg_distance_sensor == True):
        curr_dist = int(np.mean(distances[34:38]))
        print(curr_dist)
        conn.mav.distance_sensor_send(
            0,# ms Timestamp (UNIX time or time since system boot) (ignored)
            min_depth_cm,   # min_distance, uint16_t, cm
            max_depth_cm,   # min_distance, uint16_t, cm
            curr_dist,      # current_distance,	uint16_t, cm	
            0,	            # type : 0 (ignored)
            0,              # id : 0 (ignored)
            int(camera_facing_angle_degree / 45),              # orientation
            0               # covariance : 0 (ignored)
        )

# Prepare for Arducopter 4.1 3D Obstacle Avoidance
def send_obstacle_distance_3D_message():
    global mavlink_obstacle_coordinates, min_depth_cm, max_depth_cm
    global last_obstacle_distance_sent_ms
    global current_time_us
    if (enable_3D_msg_obstacle_distance == True):
        if current_time_us == last_obstacle_distance_sent_ms:
            # no new frame
            progress("no new frame")
            return
        last_obstacle_distance_sent_ms = current_time_us
        print(mavlink_obstacle_coordinates)
        current_time_us = int(round(time.time() * 1000000))
        for q in range(9):
            print("we arein the 3d loop", q)
            conn.mav.obstacle_distance_3d_send(
                current_time_us,    # us Timestamp (UNIX time or time since system boot)
                0,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                65535,
                float(mavlink_obstacle_coordinates[q][0]),
                float(mavlink_obstacle_coordinates[q][1]),
                float(mavlink_obstacle_coordinates[q][2]),
                float(min_depth_cm/100),
                float(max_depth_cm/100) #needs to be in meters
            )

def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'ROS2Mav: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)

# Request a timesync update from the flight controller, for future work.
# TODO: Inspect the usage of timesync_update 
def update_timesync(ts=0, tc=0):
    if ts == 0:
        ts = int(round(time.time() * 1000))
    conn.mav.timesync_send(tc, ts)

# Listen to ATTITUDE data: https://mavlink.io/en/messages/common.html#ATTITUDE
def att_msg_callback(value):
    global vehicle_pitch_rad
    vehicle_pitch_rad = value.pitch
    if debug_enable == 1:
        progress("INFO: Received ATTITUDE msg, current pitch is %.2f degrees" % (m.degrees(vehicle_pitch_rad),))

# Listen to flightmode data: https://mavlink.io/en/messages/common.html#HEARTBEAT
def fltmode_msg_callback(value):
    global curr_avoid_strategy
    global prev_avoid_strategy
    global enable_msg_distance_sensor
    global enable_msg_obstacle_distance
    global enable_3D_msg_obstacle_distance
    global distances, ac_version_41
    dist_arr=[0]*72
    curr_flight_mode = (value.base_mode, value.custom_mode)
    # print(curr_flight_mode)
    if ((curr_flight_mode[1] == 5) or (curr_flight_mode[1] == 2)): # Loiter and AltHold only
        curr_avoid_strategy="simple_avoid"
        if (curr_avoid_strategy != prev_avoid_strategy):
            distances[distances>0]=0
            enable_msg_distance_sensor = True
            enable_3D_msg_obstacle_distance = False
            enable_msg_obstacle_distance = False
            # send empty discance array as workaround for proyimity viewer defect
            conn.mav.obstacle_distance_send(0,0,dist_arr, 0,100,200,1,0,12)
            send_msg_to_gcs('Sending distance sensor messages to FCU')
            prev_avoid_strategy=curr_avoid_strategy

    elif ((curr_flight_mode[1] == 3) or (curr_flight_mode[1] == 4) or (curr_flight_mode[1] == 6)): # for Auto Guided and RTL modes
        curr_avoid_strategy="bendy_avoid"
        if (curr_avoid_strategy != prev_avoid_strategy):
            if (ac_version_41 == True): # only AC 4.1 or higher
                enable_msg_obstacle_distance = False
                enable_msg_distance_sensor = False
                enable_3D_msg_obstacle_distance = True
                print(enable_3D_msg_obstacle_distance)
                send_msg_to_gcs('Sending 3D obstacle distance messages to FCU')
            else:
                enable_msg_obstacle_distance = True
                enable_3D_msg_obstacle_distance = False
                enable_msg_distance_sensor = False
                send_msg_to_gcs('Sending obstacle distance messages to FCU')
            prev_avoid_strategy=curr_avoid_strategy
        
    else:
        curr_avoid_strategy="none"
        if (curr_avoid_strategy != prev_avoid_strategy):
            enable_msg_obstacle_distance = False
            enable_msg_distance_sensor = False
            enable_3D_msg_obstacle_distance = False
            send_msg_to_gcs('No valid flt mode for obstacle avoidance')
            prev_avoid_strategy=curr_avoid_strategy


# Listen to ZED ROS node for SLAM data
def zed_dist_callback(msg):
    global distances, angle_offset, increment_f, min_depth_cm, max_depth_cm
    distances=np.array([i*100 for i in msg.ranges]).astype(int)
    min_depth_cm=int(msg.range_min*100)
    max_depth_cm=int(msg.range_max*100)
    increment_f=msg.angle_increment
    # there appears to be a defect in the obstacle_distance function in Arducopter
    # so we need to set the offset manually and cant take the correct calculated increment
    increment_f=1.6
    angle_offset=msg.angle_min

def zed_9sector_callback(msg):
    global mavlink_obstacle_coordinates, min_depth_cm, max_depth_cm
    min_depth_cm=int(msg.channels[0].values[0]*100)
    max_depth_cm=int(msg.channels[0].values[1]*100)
    #print(min_depth_cm, max_depth_cm)
    for j in range(9):
        mavlink_obstacle_coordinates[j][0] = msg.points[0].x
        mavlink_obstacle_coordinates[j][1] = msg.points[1].y
        mavlink_obstacle_coordinates[j][2] = msg.points[2].z
    #print (mavlink_obstacle_coordinates)

######################################################
##  Main code starts here                           ##
######################################################

progress("INFO: Starting Vehicle communications")
conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect = True,
    source_system = 1,
    source_component = 93,
    baud=connection_baudrate,
    force_connected=True,
)

send_msg_to_gcs('Connecting to ROS node...')
rospy.init_node('listener')
rospy.Subscriber('/darknet_ros/distance_array', LaserScan, zed_dist_callback)
rospy.Subscriber('/darknet_ros/9sectorarray', PointCloud, zed_9sector_callback)
send_msg_to_gcs('ROS node connected')
sleep(1) # wait until the ROS node has booted
# register the callbacks
mavlink_callbacks = {
    #'ATTITUDE': att_msg_callback,
    'HEARTBEAT': fltmode_msg_callback,
}
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

# Send MAVlink messages in the background at pre-determined frequencies
sched = BackgroundScheduler()

sched.add_job(send_obstacle_distance_message, 'interval', seconds = 1/obstacle_distance_msg_hz, id='obst_dist')
sched.add_job(send_distance_sensor_message, 'interval', seconds = 1/obstacle_distance_msg_hz, id='dist_sensor')
sched.add_job(send_obstacle_distance_3D_message, 'interval', seconds = 1/obstacle_distance_msg_hz, id='3d_obj_dist')

sched.start()

# Begin of the main loop
last_time = time.time()
# Store the timestamp for MAVLink messages
current_time_us = int(round(time.time() * 1000000))
rospy.spin()

mavlink_thread_should_exit = True
mavlink_thread.join()
conn.close()
progress("INFO: ZED pipe and vehicle object closed.")
sys.exit(exit_code)
