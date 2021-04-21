#!/usr/bin/env python3
"""
Python 3 wrapper for identifying objects in images using darknet-yolov4 and
the point clod generated by the ZED stereo camera
Requires the ZEDSDK 3.4.2 or higher and the ZED Python API

Requires yolov4 lbdarknet.so compilation

@author: Philip Kahn, Aymeric Dujardin, Stephan Schindewolf
@date: 20210412
"""
# pylint: disable=R, W0401, W0614, W0703
import os
import sys
import time
import logging
import random
from random import randint
import math
import statistics
import getopt
from ctypes import *
import numpy as np
import cv2
import pyzed.sl as sl
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


# Sensor parameters
DEPTH_RANGE_M = [1, 20.0]       # Range for the ZED camera
depth_hfov_deg = 85

# Get the top-level logger object
log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


obstacle_line_thickness_pixel = 1 

# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# See here: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
min_depth_cm = int(DEPTH_RANGE_M[0] * 100)  # In cm
max_depth_cm = int(DEPTH_RANGE_M[1] * 100)  # In cm, should be a little conservative
distances_array_length = 72
angle_offset = None
increment_f  = None
distances = np.ones((distances_array_length), dtype=np.float) * (max_depth_cm + 1)


def sample(probs):
    s = sum(probs)
    probs = [a/s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs)-1


def c_array(ctype, values):
    arr = (ctype*len(values))()
    arr[:] = values
    return arr


class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]


class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int),
                ("uc", POINTER(c_float)),
                ("points", c_int),
                ("embeddings", POINTER(c_float)),
                ("embedding_size", c_int),
                ("sim", c_float),
                ("track_id", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]


class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]


hasGPU = True
# we are on Linux
lib = CDLL("/home/apsync/GitHub/darknet/libdarknet.so", RTLD_GLOBAL)
bridge = CvBridge()

lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

if hasGPU:
    set_gpu = lib.cuda_set_device
    set_gpu.argtypes = [c_int]

make_image = lib.make_image
make_image.argtypes = [c_int, c_int, c_int]
make_image.restype = IMAGE

get_network_boxes = lib.get_network_boxes
get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(
    c_int), c_int, POINTER(c_int), c_int]
get_network_boxes.restype = POINTER(DETECTION)

make_network_boxes = lib.make_network_boxes
make_network_boxes.argtypes = [c_void_p]
make_network_boxes.restype = POINTER(DETECTION)

free_detections = lib.free_detections
free_detections.argtypes = [POINTER(DETECTION), c_int]

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

network_predict = lib.network_predict
network_predict.argtypes = [c_void_p, POINTER(c_float)]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

load_net_custom = lib.load_network_custom
load_net_custom.argtypes = [c_char_p, c_char_p, c_int, c_int]
load_net_custom.restype = c_void_p

do_nms_obj = lib.do_nms_obj
do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

do_nms_sort = lib.do_nms_sort
do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

rgbgr_image = lib.rgbgr_image
rgbgr_image.argtypes = [IMAGE]

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)


def array_to_image(arr):
    import numpy as np
    # need to return old values to avoid python freeing memory
    arr = arr.transpose(2, 0, 1)
    c = arr.shape[0]
    h = arr.shape[1]
    w = arr.shape[2]
    arr = np.ascontiguousarray(arr.flat, dtype=np.float32) / 255.0
    data = arr.ctypes.data_as(POINTER(c_float))
    im = IMAGE(w, h, c, data)
    return im, arr


def classify(net, meta, im):
    out = predict_image(net, im)
    res = []
    for i in range(meta.classes):
        if altNames is None:
            name_tag = meta.names[i]
        else:
            name_tag = altNames[i]
        res.append((name_tag, out[i]))
    res = sorted(res, key=lambda x: -x[1])
    return res


def detect(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45, debug=False):
    """
    Performs the detection
    """
    custom_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    custom_image = cv2.resize(custom_image, (lib.network_width(
        net), lib.network_height(net)), interpolation=cv2.INTER_LINEAR)
    im, arr = array_to_image(custom_image)
    num = c_int(0)
    pnum = pointer(num)
    predict_image(net, im)
    dets = get_network_boxes(
        net, image.shape[1], image.shape[0], thresh, hier_thresh, None, 0, pnum, 0)
    num = pnum[0]
    if nms:
        do_nms_sort(dets, num, meta.classes, nms)
    res = []
    if debug:
        log.debug("about to range")
    for j in range(num):
        for i in range(meta.classes):
            if dets[j].prob[i] > 0:
                b = dets[j].bbox
                if altNames is None:
                    name_tag = meta.names[i]
                else:
                    name_tag = altNames[i]
                res.append((name_tag, dets[j].prob[i], (b.x, b.y, b.w, b.h), i))
    res = sorted(res, key=lambda x: -x[1])
    free_detections(dets, num)
    return res


netMain = None
metaMain = None
altNames = None


def get_object_depth(depth, bounds):
    '''
    Calculates the median x, y, z position of top slice(area_div) of point cloud
    in camera frame.
    Arguments:
        depth: Point cloud data of whole frame.
        bounds: Bounding box for object in pixels.
            bounds[0]: x-center
            bounds[1]: y-center
            bounds[2]: width of bounding box.
            bounds[3]: height of bounding box.

    Return:
        x, y, z: Location of object in meters.
    '''
    area_div = 2

    x_vect = []
    y_vect = []
    z_vect = []

    for j in range(int(bounds[0] - area_div), int(bounds[0] + area_div)):
        for i in range(int(bounds[1] - area_div), int(bounds[1] + area_div)):
            z = depth[i, j, 2]
            if not np.isnan(z) and not np.isinf(z):
                x_vect.append(depth[i, j, 0])
                y_vect.append(depth[i, j, 1])
                z_vect.append(z)
    try:
        x_median = statistics.median(x_vect)
        y_median = statistics.median(y_vect)
        z_median = statistics.median(z_vect)
    except Exception:
        x_median = -1
        y_median = -1
        z_median = -1
        pass

    return x_median, y_median, z_median


def generate_color(meta_path):
    '''
    Generate random colors for the number of classes mentioned in data file.
    Arguments:
    meta_path: Path to .data file.

    Return:
    color_array: RGB color codes for each class.
    '''
    random.seed(42)
    with open(meta_path, 'r') as f:
        content = f.readlines()
    class_num = int(content[0].split("=")[1])
    color_array = []
    for x in range(0, class_num):
        color_array.append((randint(0, 255), randint(0, 255), randint(0, 255)))
    return color_array


# Calculate the distances array by dividing the FOV (horizontal) into $distances_array_length rays,
# then pick out the depth value at the pixel corresponding to each ray. Based on the definition of
# the MAVLink messages, the invalid distance value (below MIN/above MAX) will be replaced with MAX+1.
#    
# [0]    [35]   [71]    <- Output: distances[72]
#  |      |      |      <- step = width / 72
#  ---------------      <- horizontal line, or height/2
#  \      |      /
#   \     |     /
#    \    |    /
#     \   |   /
#      \  |  /
#       \ | /           
#       Camera          <- Input: depth_mat, obtained from depth image
#
# Note that we assume the input depth_mat is already processed by at least hole-filling filter.
# Otherwise, the output array might not be stable from frame to frame.
def distances_from_depth_image(point_cloud_mat, distances, min_depth_m, max_depth_m, width, height, obstacle_line_thickness_pixel):
    # Parameters for depth image
    depth_img_width  = width*2

    # Parameters for obstacle distance message
    step = depth_img_width / distances_array_length
    
    for i in range(distances_array_length):

        dist_arr=[]
        # do the depth sensing at each individual segment and take the mean value over the obstacle line thickness
        for j in range(int(obstacle_line_thickness_pixel)):
            err, point_cloud_value = point_cloud_mat.get_value((int(i*step)), ((int(height-obstacle_line_thickness_pixel/2))+j))
            dist_m = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])
            dist_arr.append(dist_m)
            #Remove nan and inf values
            dist_arr = [x for x in dist_arr if math.isnan(x) == False]
            dist_arr = [x for x in dist_arr if math.isinf(x) == False]

        #determine mean value over obstacle line thickness
        dist_m=np.mean(dist_arr)

        # Default value, unless overwritten: 
        #   A value of max_distance + 1 (cm) means no obstacle is present. 
        #   A value of UINT16_MAX (65535) for unknown/not used.
        distances[i] = 655.35

        if dist_m >= min_depth_m and dist_m <= max_depth_m:
            distances[i] = dist_m
        elif dist_m < min_depth_m:
            distances[i] = 0
        elif np.isnan(dist_m):
            distances[i] = 0

    #print ("final distances array in m: ", distances)


def main(argv):

    thresh = 0.25
    darknet_path="/home/apsync/GitHub/darknet/"
    config_path = darknet_path + "cfg/yolov4-tiny.cfg"
    weight_path = darknet_path + "yolov4-tiny.weights"
    meta_path = darknet_path + "cfg/coco.data"
    svo_path = None
    zed_id = 0

    help_str = 'darknet_zed.py -c <config> -w <weight> -m <meta> -t <threshold> -s <svo_file> -z <zed_id>'
    try:
        opts, args = getopt.getopt(
            argv, "hc:w:m:t:s:z:", ["config=", "weight=", "meta=", "threshold=", "svo_file=", "zed_id="])
    except getopt.GetoptError:
        log.exception(help_str)
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            log.info(help_str)
            sys.exit()
        elif opt in ("-c", "--config"):
            config_path = arg
        elif opt in ("-w", "--weight"):
            weight_path = arg
        elif opt in ("-m", "--meta"):
            meta_path = arg
        elif opt in ("-t", "--threshold"):
            thresh = float(arg)
        elif opt in ("-s", "--svo_file"):
            svo_path = arg
        elif opt in ("-z", "--zed_id"):
            zed_id = int(arg)

    input_type = sl.InputType()
    if svo_path is not None:
        log.info("SVO file : " + svo_path)
        input_type.set_from_svo_file(svo_path)
    else:
        # Launch camera by id
        input_type.set_from_camera_id(zed_id)

    init = sl.InitParameters(input_t=input_type)
    init.coordinate_units = sl.UNIT.METER
    
    # Start the ROS nodes for the yolo image and the depth information
    rospy.init_node('darknet_ros')
    rospy.loginfo('darknet yolo node started')
    pub = rospy.Publisher('/darknet_ros/detection_image', Image, queue_size=10)
    pub2 = rospy.Publisher('/darknet_ros/distance_array', LaserScan, queue_size=50)
    pub3 = rospy.Publisher('/darknet_ros/color_image', Image, queue_size=10)


    cam = sl.Camera()
    if not cam.is_opened():
        log.info("Opening ZED Camera...")
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        log.error(repr(status))
        exit()

    runtime = sl.RuntimeParameters()
    # Use STANDARD sensing mode
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD
    mat = sl.Mat()
    mat_lv = sl.Mat()
    point_cloud_mat = sl.Mat()

    # Import the global variables. This lets us instance Darknet once,
    # then just call performDetect() again without instancing again
    global metaMain, netMain, altNames  # pylint: disable=W0603
    assert 0 < thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(config_path):
        raise ValueError("Invalid config path `" +
                         os.path.abspath(config_path)+"`")
    if not os.path.exists(weight_path):
        raise ValueError("Invalid weight path `" +
                         os.path.abspath(weight_path)+"`")
    if not os.path.exists(meta_path):
        raise ValueError("Invalid data file path `" +
                         os.path.abspath(meta_path)+"`")
    if netMain is None:
        netMain = load_net_custom(config_path.encode(
            "ascii"), weight_path.encode("ascii"), 0, 1)  # batch size = 1
    if metaMain is None:
        metaMain = load_meta(meta_path.encode("ascii"))
    if altNames is None:
        # In thon 3, the metafile default access craps out on Windows (but not Linux)
        # Read the names file and create a list to feed to detect
        try:
            with open(meta_path) as meta_fh:
                meta_contents = meta_fh.read()
                import re
                match = re.search("names *= *(.*)$", meta_contents,
                                  re.IGNORECASE | re.MULTILINE)
                if match:
                    result = match.group(1)
                else:
                    result = None
                try:
                    if os.path.exists(result):
                        with open(result) as names_fh:
                            names_list = names_fh.read().strip().split("\n")
                            altNames = [x.strip() for x in names_list]
                except TypeError:
                    pass
        except Exception:
            pass

    color_array = generate_color(meta_path)

    # get parameters for depth sensing
    width = round(cam.get_camera_information().camera_resolution.width/2)
    height = round(cam.get_camera_information().camera_resolution.height/2)
    res = sl.Resolution()
    res.width = width
    res.height = height
    angle_offset = 0 - (depth_hfov_deg / 2)
    increment_f = depth_hfov_deg / distances_array_length

    #Initialize laserscan node
    scan = LaserScan()
    scan.header.frame_id = 'zed_horizontal_scan'
    scan.angle_min = angle_offset
    scan.angle_max = depth_hfov_deg/2
    scan.angle_increment = increment_f
    scan.range_min = DEPTH_RANGE_M[0]
    scan.range_max = DEPTH_RANGE_M[1]
    scan.intensities = []
    scan.time_increment = 0
    fps = 1

    while not rospy.is_shutdown():
        start_time = time.time() # start time of the loop
        current_time = rospy.Time.now()
        err = cam.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(mat, sl.VIEW.LEFT)
            image = mat.get_data()
            cam.retrieve_image(mat_lv, sl.VIEW.LEFT)
            image_lv = mat_lv.get_data()
            overlay = image_lv
            scan.header.stamp = current_time
            scan.scan_time = fps


            cam.retrieve_measure(
                point_cloud_mat, sl.MEASURE.XYZRGBA)
            depth = point_cloud_mat.get_data()

            print("\033[0;0H")
    
            distances_from_depth_image(point_cloud_mat, distances, DEPTH_RANGE_M[0], DEPTH_RANGE_M[1], width, height, obstacle_line_thickness_pixel)
            scan.ranges = distances
            array_midpoint=int(distances_array_length/2)
            distance_center=distances[array_midpoint]

            # Publish the distance information for the mavros node
            pub2.publish(scan)

            # provide distance info to video stream
            # Draw a horizontal line to visualize the obstacles' line
            x1, y1 = int(0), int(height)
            x2, y2 = int(width*2), int(height)
            line_thickness = obstacle_line_thickness_pixel
            cv2.line(image_lv, (x1, y1), (x2, y2), (0, 255, 0), thickness=line_thickness)

            #print("Distance to Camera at position {},{} (image center): {:1.3} m".format(width, height, distance_center))
            if distance_center > DEPTH_RANGE_M[1]:
                cv2.circle(image_lv, (width, height), 20, (0, 255, 0), 2)
            elif distance_center <= DEPTH_RANGE_M[1] and distance_center > 10:
                cv2.putText(image_lv, ("{:1.3} m".format(distance_center)), ((width-50), (height+50)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.circle(image_lv, (width, height), 20, (0, 255, 0), 2)
            elif distance_center > 5 and distance_center <= 10:
                cv2.putText(image_lv, ("{:1.3} m".format(distance_center)), ((width-70), (height+65)), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 2)
                cv2.circle(image_lv, (width, height), 20, (0, 255, 255), 4)
            else:
                cv2.putText(image_lv, ("{:1.3} m".format(distance_center)), ((width-100), (height+70)), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 2)
                cv2.circle(image_lv, (width, height), 20, (0, 0, 255), -1)
                cv2.rectangle(image_lv, (0,100), ((width*2-5),(height*2)), (30,30,255), 3)

            # Do the yolo object detection
            detections = detect(netMain, metaMain, image, thresh)

            print(chr(27) + "[2J"+"**** " + str(len(detections)) + " Detection Results ****")
            for detection in detections:
                label = detection[0]
                confidence = detection[1]
                pstring = label+": "+str(np.rint(100 * confidence))+"%"
                print(pstring)
                bounds = detection[2]
                y_extent = int(bounds[3])
                x_extent = int(bounds[2])
                # Coordinates are around the center
                x_coord = int(bounds[0] - bounds[2]/2)
                y_coord = int(bounds[1] - bounds[3]/2)
                #boundingBox = [[x_coord, y_coord], [x_coord, y_coord + y_extent], [x_coord + x_extent, y_coord + y_extent], [x_coord + x_extent, y_coord]]
                thickness = 1
                x, y, z = get_object_depth(depth, bounds)
                distance = math.sqrt(x * x + y * y + z * z)
                distance = "{:.2f}".format(distance)
                cv2.rectangle(image, (x_coord - thickness, y_coord - thickness),
                              (x_coord + x_extent + thickness, y_coord + (18 + thickness*4)),
                              color_array[detection[3]], -1)
                cv2.putText(image, pstring + " " +  (str(distance) + " m"),
                            (x_coord + (thickness * 4), y_coord + (10 + thickness * 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.rectangle(image, (x_coord - thickness, y_coord - thickness),
                              (x_coord + x_extent + thickness, y_coord + y_extent + thickness),
                              color_array[detection[3]], int(thickness*2))

            # convert image to ros
            imgMsg = bridge.cv2_to_imgmsg(image, encoding="bgra8")
            imgMsg_lv = bridge.cv2_to_imgmsg(image_lv, encoding="bgra8")
            # publish images to ros nodes
            pub.publish(imgMsg)
            pub3.publish(imgMsg_lv)
        fps = (time.time() - start_time)
        print("\033[0;0H"+"FPS: {:1.3}".format(1.0 / fps))
        #rospy.Rate(1.0).sleep()  # 1 Hz
    #cv2.destroyAllWindows()

    cam.close()
    log.info("\nFINISH")


if __name__ == "__main__":
    main(sys.argv[1:])
