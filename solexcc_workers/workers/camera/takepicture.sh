#!/bin/bash 
# set -x

picture_camera() {
	if ! ps -ef | grep [i]mage:=/darknet_ros/color_image
	then
           rosrun image_view image_saver image:=/darknet_ros/color_image _save_all_image:=false _filename_format:=/home/apsync/dflogger/dataflash/zed_image/zed_left%04i.jpg __name:=zed_image_saver > /dev/null &
	   sleep 6
        fi
	rosservice call /zed_image_saver/save 
	if [ $? -eq 0 ]
		then
			echo "taken"
			exit 0
	else
                        echo "err" >&2
			exit 1
	fi
	}

picture_depth() {
        if ! ps -ef | grep [d]arknet_ros/nine_sector_image
        then
           rosrun image_view image_saver image:=/darknet_ros/nine_sector_image _save_all_image:=false _filename_format:=/home/apsync/dflogger/dataflash/zed_image/zed_depthgray%04i.jpg __name:=zed_depth_image_saver > /dev/null &
           sleep 6
        fi
        rosservice call /zed_depth_image_saver/save > /dev/null
	if [ $? -eq 0 ]
                then
                        echo "depth image taken"
			exit 0
        else
                        echo "err" >&2
                        exit 1
        fi
        }

picture_yolo() {
        if ! ps -ef | grep [d]arknet_ros/detection_image
        then
           rosrun image_view image_saver image:=/darknet_ros/detection_image _save_all_image:=false _filename_format:=/home/apsync/dflogger/dataflash/zed_image/yolo_image%04i.jpg __name:=yolo_image_saver > /dev/null &
           sleep 6
        fi
        rosservice call /yolo_image_saver/save > /dev/null
        if [ $? -eq 0 ] 
                then    
                        echo "yolo image taken"
			exit 0
	else
                        echo "err" >&2
			exit 1
        fi
        }

picture_depth_map() {
	if ! ps -ef | grep [s]tereo_dnn_ros_viz/output
        then
           rosrun image_view image_saver image:=/stereo_dnn_ros_viz/output _save_all_image:=false _filename_format:=/home/apsync/dflogger/dataflash/zed_image/col_depth%04i.jpg __name:=depth_color_image_saver > /dev/null &
	   sleep 6
	fi
        rosservice call /depth_color_image_saver/save > /dev/null
	if [ $? -eq 0 ] 
                then    
                        echo "complete image set taken"
			exit 0
	else
                        echo "err" >&2
			exit 1
        fi
	}

stopimage() {
    sudo pkill -f "image_view" 
    exit 1
}

case "$1" in 
    real)   picture_camera ;;
    depth)   picture_depth ;;
    yolo)   picture_yolo ;;
    color)   picture_depth_map ;;
    stop)    stopimage ;;
    *) echo "usage: $0 take_picture|take_depth_pic|stop" >&2
       exit 1
       ;;
esac
exit 0
