# 
## Setup 
Install ROS Melodic
On your Jetson Nano using this [link](
https://www.stereolabs.com/blog/ros-and-nvidia-jetson-nano/) as a reference 

sudo apt-get install \
ros-*-realsense2-camera \
ros-*-move-base \
ros-*-pointcloud-to-laserscan \
ros-*-imu-filter-madgwick \
ros-*-robot-localization \
ros-*-amcl \
ros-*-slam-gmapping \
ros-*-joy \
ros-*-teleop_twist_joy \

install [ros android driver](https://play.google.com/store/apps/details?id=org.ros.android.sensors_driver&hl=en_US&gl=US)


https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/

librealsense2 Is it a dependency for the real sense cameras and while can be installed through the pseudo-app to get I ran into issues doing so. I said recommend using this link to do so following method 2
check using realsense-viewer
Establish GPIO permissions