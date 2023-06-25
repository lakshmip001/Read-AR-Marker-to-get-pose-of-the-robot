# Read-AR-Marker-to-get-pose-of-the-robot
- Use the camera topic to read the AR marker and get the pose of the robot (https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html).
used camera_info topic to get the camera matrix.
Obtained  the pose of the markers using the tutorial given in the link.
- Navigate using cmd_vel topic to a position exactly below the centre of the marker from wherever the robot starts. 
computed orbit for reaching the center.
Navigated the robot to the by using /cmd_topic .
To setup simulation, Kindly download turtlebot3 packages & replace with the .urdf file as given in the instructions in catkin workspace & place this package inside the catkin workspace:

roscore
roslaunch my_assignment turtlebot3_qr.launch.
