# ws

## Installation
Installations for nav2 and zed wrapper 

### Install nav2-bringup
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-navigation2

if it shoots our errors during installations, run:

sudo apt install libncurses-dev=6.3-2 libtinfo6=6.3-2 libncurses6=6.3-2 libncursesw6=6.3-2

then rerun the above 3 commands
```

### Install zed_ros2_wrapper
```bash
# Move to the `src` folder of the ROS 2 Workspace
cd ~/ros2_ws/src/ 
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
# Install the required dependencies 
rosdep install --from-paths src --ignore-src -r -y

[If rosdep is not installed, 
run the following command to install rosdep and then rerun the above command:

sudo apt install python3-rosdep2
rosdep update 
rosdep install --from-paths src --ignore-src -r -y]

# Build the wrapper
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release OR just colcon build 
[Note: you will need to install the zed2 SDK if you have not already]
# Source the environment variables
source install/setup.bash
# Run the pkgs

```
