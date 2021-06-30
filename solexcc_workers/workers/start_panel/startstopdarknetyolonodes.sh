#!/bin/bash -c 

startdnn() {
    if ps -ef | grep [d]arknet_ros
    then
      echo "YOLO already running" >&2
      exit 1
    fi

    roslaunch darknet_ros darknet_ros.launch > /dev/null & 
    sleep 3
    echo "YOLO node launched"
}

stopdnn() {
    pkill -f "darknet_ros" 
    echo "YOLO node shutdown"
}

# YOLO requires trailnet or stereoDNN launched first
if ! ps -ef | grep [s]tereo_dnn_ros && ! ps -ef | grep [c]affe_ros && ! ps -ef | grep [d]arknet_ros
   then
      echo "Either StereoDNN or TrailnetDNN must be launched first" >&2
      exit 1
fi

case "$1" in 
    start)   startdnn ;;
    stop)    stopdnn ;;
    *) echo "usage: $0 start|stop" >&2
       exit 1
       ;;
esac
