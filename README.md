# 
## Setup 
Install ROS Melodic
On your Jetson Nano using this [link](
https://www.stereolabs.com/blog/ros-and-nvidia-jetson-nano/) as a reference 
 
sudo apt-get install \
ros-*-realsense2-camera \
ros-*-slam-gmapping \
ros-*-teleop_twist_joy \
ros-*-joy \
ros-*-move-base \
ros-*-pointcloud-to-laserscan \


ros-*-explore-lite \
ros-*-amcl \

librealsense2 Is it a pendency for the real sense cameras and while can be installed through the pseudo-app to get I ran into issues doing so. I said recommend using this link to do so following method 2
check using realsense-viewer
Establish GPIO permissions