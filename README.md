# aruco_mavconn
Positioning system based on Aruco Boards for Ardupilot

aruco_mavconn is the porting to (non-ROS) C/C++ of the positioning system based on Aruco Boards described in this wiki page (https://ardupilot.org/dev/docs/ros-aruco-detection.html), it depends on OpenCV and MAVCONN.

MAVCONN library (https://github.com/mavlink/mavros/tree/master/libmavconn) is the mavlink connection and communication library used in MAVROS that could be used also outside the ROS environment.

The intent was that of learning how to use libmavconn and a little bit more of OpenCV while at the same time obtaining a lighter application to run on Single Board Computer.

More info on the related Ardupilot blog post (https://discuss.ardupilot.org/t/using-mavconn-library-for-sending-non-gps-navigation-messages-in-c-c/54105)
