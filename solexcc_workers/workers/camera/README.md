# ZED Camera Control

The `camera` worker provides the camera panel, and provides a means of taking a picture (`take_picture`), starting/stopping video (`toggle_video`), and controlling various settings on the  camera. If an error occurs, the `speech` message is used to communicate the nature of the problem.
It offers the following capabilities and controls for the ZED camera on the flight screen:

- Take picture: a jpg image is stored on the TX2 SD card. 
- Start/Stop video toggle: record a MP4 stream on the SD card.
- Selector for normal `real` pictures or video or to record the color-coded depth map which is being calculated byhe stereoDNN network.
- Camera settings (not yet implemented)


Ensure that you have properly mounted an SD card and configured it as where you store the dataflash logs. You may adopt the storage path by editing the .sh scripts in this folder.




