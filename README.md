# 
## Setup 
Install ROS Melodic
On your Jetson Nano using this [link](
https://www.stereolabs.com/blog/ros-and-nvidia-jetson-nano/) as a reference 

sudo apt-get install \
ros-melodic-realsense2-camera \
ros-melodic-move-base \
ros-melodic-pointcloud-to-laserscan \
ros-melodic-imu-filter-madgwick \
ros-melodic-robot-localization \
ros-melodic-slam-gmapping \
ros-melodic-amcl \
ros-melodic-joy \
ros-melodic-teleop_twist_joy

https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/

librealsense2 Is it a dependency for the real sense cameras and while can be installed through the pseudo-app to get I ran into issues doing so. I said recommend using this link to do so following method 2
check using realsense-viewer
Establish GPIO permissions