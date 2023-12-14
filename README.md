# shutter-gesture-control

## User Manuel
### Cloning Repo
Before cloning, go to Settings >> Developer Settings >> Personal access tokens to create a token for using Git over HTTPS. More documentations: https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#cloning-with-https-urls
```
git clone https://github.com/ziyaosg/shutter_gesture_control.git
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
```

### Building Project
Make sure you have the required package structure for the project to build correctly. More documentations: https://wiki.ros.org/ROS/Tutorials/CreatingPackage
```
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```


### Run Shutter Gesture Control (with launch file)
```
# start ROS Master
roscore

# start realsense camera
roslaunch realsense2_camera rs_camera.launch

# run launch file
roslaunch shutter-gesture-control shutter_gesture_control.launch

# run control script
rosrun shutter_gesture_control gesture_control.py
```


### Run Shutter Gesture Control (without launch file)
More documentations: https://shutter-ros.readthedocs.io/en/latest/packages/shutter_bringup.html.
```
# start ROS Master
roscore

# start realsense camera
roslaunch realsense2_camera rs_camera.launch

# start kinect body tracking
roslaunch azure_kinect_ros_driver driver.launch body_tracking_enabled:=true overwrite_robot_description:=false

# start shutter
roslaunch shutter_bringup shutter_with_face.launch

# send service request for manually starting the JointGroupPositionController
rosservice call /controller_manager/switch_controller "start_controllers: [joint_group_controller]
strictness: 1"

# reposition shutter to neutral
rostopic pub -1 /joint_group_controller/command std_msgs/Float64MultiArray 'data: [0.0, 0.0, 0.0, 0.0]'

# run control script
rosrun shutter_gesture_control gesture_control.py
```
