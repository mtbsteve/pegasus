#!/bin/bash  
# set -x

#startvideo() {
#rosrun image_view video_recorder image:=/zed/zed_node/left/image_rect_color _filename:=/home/apsync/dflogger/dataflash/zed_video/zed_video`ls /home/apsync/dflogger/dataflash/zed_video/zed_video* | wc -l`.avi _codec:="I420" > /dev/null & 
#	}

startvideo() {
rosrun image_view video_recorder image:=/darknet_ros/color_image _filename:=/home/apsync/dflogger/dataflash/zed_video/zed_video`ls /home/apsync/dflogger/dataflash/zed_video/zed_video* | wc -l`.avi _codec:="I420" > /dev/null & 
	}
startvideo_depth() {
rosrun image_view video_recorder image:=/darknet_ros/nine_sector_image _filename:=/home/apsync/dflogger/dataflash/zed_video/zed_9secvid`ls /home/apsync/dflogger/dataflash/zed_video/zed_con_map_vid* | wc -l`.avi _codec:="I420" > /dev/null &
        }

startvideo_yolo() {
rosrun image_view video_recorder image:=/darknet_ros/detection_image _filename:=/home/apsync/dflogger/dataflash/zed_video/zed_yolo_video`ls /home/apsync/dflogger/dataflash/zed_video/zed_yolo_video* | wc -l`.avi _codec:="I420" > /dev/null &
        }

startvideo_all() {
rosrun image_view video_recorder image:=/stereo_dnn_ros_viz/output/ _filename:=/home/apsync/dflogger/dataflash/zed_video/zed_all_video`ls /home/apsync/dflogger/dataflash/zed_video/zed_all_video* | wc -l`.avi _codec:="MJPG" > /dev/null & 
	}

stopvideo() {
    pkill -f "video_recorder" 
}

case "$1" in 
    real)   startvideo ;;
    depth)   startvideo_depth ;;
    yolo)   startvideo_yolo ;;
    color)   startvideo_all ;;
    stop)    stopvideo ;;
    *) echo "usage: $0 real|depth|yolo|color|stop" >&2
       exit 1
       ;;
esac
