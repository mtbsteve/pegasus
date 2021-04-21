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
from sensor_msgs.msg import LaserScan

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

connection_string_default = '127.0.0.1:14855'
connection_baudrate_default = 1500000

# Use this to rotate all processed data
camera_facing_angle_degree = 0

# Store device serial numbers of connected camera
device_id = None

# Enable/disable each message/function individually
enable_msg_obstacle_distance = True
enable_msg_distance_sensor = False
obstacle_distance_msg_hz_default = 15.0

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
camera_name = args.camera_name

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
    if current_time_us == last_obstacle_distance_sent_ms:
        # no new frame
        return
    last_obstacle_distance_sent_ms = current_time_us
    if angle_offset is None or increment_f is None:
        progress("Please call set_obstacle_distance_params before continue")
    else:
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

# https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
def send_single_distance_sensor_msg(distance, orientation):
    # Average out a portion of the centermost part
    conn.mav.distance_sensor_send(
        0,                  # ms Timestamp (UNIX time or time since system boot) (ignored)
        min_depth_cm,       # min_distance, uint16_t, cm
        max_depth_cm,       # min_distance, uint16_t, cm
        distance,           # current_distance,	uint16_t, cm	
        0,	                # type : 0 (ignored)
        0,                  # id : 0 (ignored)
        orientation,        # orientation
        0                   # covariance : 0 (ignored)
    )

# https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
def send_distance_sensor_message():
    global distances
    # Average out a portion of the centermost part
    curr_dist = int(np.mean(distances[33:38]))
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

def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'D4xx: ' + text_to_be_sent
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

# Listen to AHRS2 data: https://mavlink.io/en/messages/ardupilotmega.html#AHRS2
def ahrs2_msg_callback(value):
    global vehicle_pitch_rad
    vehicle_pitch_rad = value.pitch
    if debug_enable == 1:
        progress("INFO: Received AHRS2 msg, current pitch is %.2f degrees" % (m.degrees(vehicle_pitch_rad)))

def zed_dist_callback(msg):
    print(msg.header)
    #print(msg.header.time)
    #print(msg.header.frame_id)
    print(msg.angle_min)
    print(msg.angle_max)
    print(msg.angle_increment)
    print(msg.time_increment)
    print(msg.scan_time)
    print(msg.range_min)
    print(msg.range_max)
    print(msg.ranges)
    print(msg.intensities)



######################################################
##  Main code starts here                           ##
######################################################

rospy.init_node('listener')
rospy.loginfo('listener node started')
rospy.Subscriber('/darknet_ros/distance_array', LaserScan, zed_dist_callback)


progress("INFO: Starting Vehicle communications")
conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect = True,
    source_system = 1,
    source_component = 93,
    baud=connection_baudrate,
    force_connected=True,
)
mavlink_callbacks = {
    'ATTITUDE': att_msg_callback,
}
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

send_msg_to_gcs('Connecting to camera...')
send_msg_to_gcs('Camera connected.')

# Send MAVlink messages in the background at pre-determined frequencies
sched = BackgroundScheduler()

if enable_msg_obstacle_distance:
    sched.add_job(send_obstacle_distance_message, 'interval', seconds = 1/obstacle_distance_msg_hz)
    send_msg_to_gcs('Sending obstacle distance messages to FCU')
elif enable_msg_distance_sensor:
    sched.add_job(send_distance_sensor_message, 'interval', seconds = 1/obstacle_distance_msg_hz)
    send_msg_to_gcs('Sending distance sensor messages to FCU')
else:
    send_msg_to_gcs('Nothing to do. Check params to enable something')
    pipe.stop()
    conn.mav.close()
    progress("INFO: Realsense pipe and vehicle object closed.")
    sys.exit()

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
