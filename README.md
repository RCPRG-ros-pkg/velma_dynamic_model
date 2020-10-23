# Dynamic model of Velma

This package contains dynamic model of Velma robot. It uses [DART physics library](https://dartsim.github.io), version 6.7.2.
It is the same dynamic model as one used in Gazebo simulation within simulated control system, except collision checking is disabled.

It requires packages with URDF models of the robot:
* [velma_description](https://github.com/RCPRG-ros-pkg/velma_robot.git)
* [lwr_defs](https://github.com/RCPRG-ros-pkg/lwr_robot.git)
* [barrett_hand_defs](https://github.com/RCPRG-ros-pkg/barrett_hand_robot.git)

## Installation

If you have workspace for Velma installed [https://github.com/RCPRG-ros-pkg/RCPRG_rosinstall](https://github.com/RCPRG-ros-pkg/RCPRG_rosinstall), just clone this repo into src and build.

For stand-alone workspace run the folowing commands:

```bash
mkdir -p ~/ws_velma_dyn/src
cd ~/ws_velma_dyn/src
git clone https://github.com/RCPRG-ros-pkg/velma_dynamic_model.git
git clone https://github.com/RCPRG-ros-pkg/velma_robot.git
git clone https://github.com/RCPRG-ros-pkg/lwr_robot.git
git clone https://github.com/RCPRG-ros-pkg/barrett_hand_robot.git
```
If you don't have DART 6.7.2 installed:
```bash
git clone -b v6.7.2 https://github.com/dartsim/dart.git
```

Then, configure and build workspace:
```bash
cd ~/ws_velma_dyn
source /opt/ros/melodic/setup.bash
catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF
catkin build
```
## Running the test program

Here we assume, the workspace is in ~/ws_velma_dyn.
Open a new terminal and type commands:
```bash
source ~/ws_velma_dyn/devel/setup.bash
roscore
```
In the second terminal:
```bash
source ~/ws_velma_dyn/devel/setup.bash
roslaunch velma_dynamic_model velma_dynamic_model_test.launch
```
The last command should load URDF model of Velma robot into ROS param, run rviz and the test program.
