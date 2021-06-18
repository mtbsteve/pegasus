# Select Video Stream
With this worker, you can switch between the available ROS image topics.
To get an overview on all available image topics, run rostopic list. You may define a RSP video stream by adding a RTSP video node in the file $CATKIN_WS/src/ros_rtsp/config/stream_setup.yaml.
Then add the defined nodes into the worker.js file like:
```
......
function getFeatures() {
    d("getFeatures()");
    
    // Return a single feature, or multiple
    return {
        video: { 
            supported: true,
            endpoints: [
                {
                    name: "ZED-Video ROS node", 
                    type: "rtsp",
                    url: "rtsp://10.0.1.128:8554/zedimage" // include your node here
                },
                {
                    name: "ZED-YOLO ROS node", 
                    type: "rtsp",
                    url: "rtsp://10.0.1.128:8554/zedyolo"
                }
            ]
        }
    };
}
.........
```
