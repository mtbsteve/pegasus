#!/bin/bash -c 

startdnn() {
if ps -ef | grep [c]affe_ros
   then
	roslaunch px4_controller ap_mavros_controller.launch > /dev/null &
	sleep 30
        rosrun mavros mavsys rate --all 5
        echo "MAVROS node launched"
        exit 0
   else
        echo "Trailnet is not running must be started first" >&2
	exit 1
fi
}

stopdnn() {
   pkill -f "px4_controller" 
   echo "MAVROS shutdown"
}

case "$1" in 
    start)   startdnn ;;
    stop)    stopdnn ;;
    *) echo "usage: $0 start|stop" >&2
       exit 1
       ;;
esac
