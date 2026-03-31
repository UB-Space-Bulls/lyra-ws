# Arm Description Package

This package will hold the bringup for the arm simulation and the arm operating.

## Launching the Sim
First make sure you have gazebo ignition and ros2 humble installed.

Before running make sure you are in the root of the ws:
```
colcon build && source install/setup.sh
ros2 launch armDescription armControl.launch.py
```
Then for manual control run: 
`ros2 run armDescription keyboard`

moveit2 implementation is currently in progress.


## TODO
- [ ] Implement moveit2 for path planning and execution for the arm's end effector.
- [ ] Finish & Test Motor controller.
