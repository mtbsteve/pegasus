#!/bin/bash -c 

export PYENV_ROOT="$HOME/.pyenv"
export PATH="$PYENV_ROOT/bin:$PATH"

startdnn() {
   if ps -ef | grep [z]ed_yolo
   then
      echo "ZED Camera already running" >&2
      exit 1
   fi

   roscore &
   sleep 5
   rosrun zed_yolo zed_yolo_node.py > /dev/null &
   sleep 7
   roslaunch ros_rtsp rtsp_streams.launch > /dev/null &
   echo "ZED camera launched"
}

stopdnn() {
    pkill -f "ros_rtsp"
    pkill -f "zed_yolo" 
    pkill -f "roscore" 
echo "ZED Camera shutdown"
}

case "$1" in 
    start)   startdnn ;;
    stop)    stopdnn ;;
    *) echo "usage: $0 start|stop" >&2
       exit 1
       ;;
esac
