# 2025-6  ws for UB Space Bulls Lyra rover

## Before Contributing
First of all make sure you have ros2 humble installed on Ubuntu 22.04 environment (native, vm, container, etc...).
If you do not have a ros2 environment setup follow the documentation to do so. 

### Skills (useful/required)
- 'Bash Shell' language understanding
- Knowledge of the 'apt' package manager
- fundementals in python and/or C++ (this is __NOT__ a good place to learn to code)
- understanding of the colcon workspace
- ros2 cli and client interfaces (rclpy/rclpp)
- ros2 inner workings in general (i.e. nodes and topics)
  
If you don't know these skill that doesnt mean you wont be allowed to contribute, just learn along the way and ask questions.


## Installation
Instructions for coding and building the ws:
- Clone the ws in the root of your home directory.
- 



### Install nav2-bringup
```bash
sudo apt update
sudo apt install libncurses-dev=6.3-2 libtinfo6=6.3-2 libncurses6=6.3-2 libncursesw6=6.3-2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-navigation2
```

### Install zed_ros2_wrapper
Follow the instructions [here](https://github.com/stereolabs/zed-ros2-wrapper). Make sure you have a cuda capable gpu if installing on home machine.
