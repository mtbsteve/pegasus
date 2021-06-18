#!/bin/bash -c 

startdnn() {
if !(ps -ef | grep [r]oscore)
   then
      echo "ZED node not yet initiated" >&2
      exit 1
fi

rosrun zed_yolo zed_ros_to_mavlink.py > /dev/null &
echo "ROS2Mavlink launched"
}

stopdnn() {
    pkill -f "zed_ros" 
echo "Trailnet node shutdown"
}

case "$1" in 
    start)   startdnn ;;
    stop)    stopdnn ;;
    *) echo "usage: $0 start|stop" >&2
       exit 1
       ;;
esac
