# ECE470-Project

## Installing the simulator

In order to install the simulator follow the instructions on the mbzirc git repo.
https://github.com/osrf/mbzirc

Officially the simulator supports ubuntu 20.04 and ROS2 galactic, however, Blerim was able to get this to run on ROS2 humble, with ubuntu 22.04

Installing this package:
1. Simply clone this repo into your `~/workspace/src`, under the name boat_control.
2. Build the boat control package with `colcon build --packages-select boat_control --merge-install`

To run the simulator joystick demo:

In seperate terminals run each of the following commands

0. run `source /opt/ros/galactic/setup.sh`. If you get a message regarding another ros version set up, run the previous command again.
1. ` ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r test/simple_arm_demo.sdf"`
2. `ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=simple_demo model:=usv x:=12.75 y:=0 z:=0.3 R:=0 P:=0 Y:=1.571 arm:=mbzirc_simple_arm gripper:=mbzirc_suction_gripper`
3. `ros2 run joy joy_node`
4. `ros2 run boat_control joyctl`
5. You can now use any of the joysticks supported here https://index.ros.org/p/joy/ and you will be able to move the left and right thrusters

TODO:
1. Create a launch file for the joy demo
2. Waypoint navigation with ground truth coordinates

Inverse Kinematic Equations (y,z) -> (θ<sub>0</sub>,θ<sub>1</sub>):

[Note: L = 0.75, H = $\sqrt(y^2 + z^2)$]

1. θ<sub>0</sub> = $\sin^-1(z/H) + cos^-1(H/(2L))$
2. θ<sub>1</sub> = $\pi/2 + sin^-1(z/H) - cos^-1(H/(2L))$


## Setting up the simulator
To set up this environment we modified the existing demo are test included in the mbzirc original simulator.
We have included this sdf file within within this repo although you will need to place it in the mbzirc_ign/worlds/test directory.
### Sea state
In order for us to be able to pick up a target we needed to calm the ocean waters since the waves would throw off our inverse kinematics.
Information about how sea state works is included here https://en.wikipedia.org/wiki/Sea_state
 1. `cd src/mbzirc/mbzirc_ign/scripts`
 2. `python3 set_sea_state.py --sea-state 0`
 3. `colcon build --merge-install --cmake-clean-cache --packages-select mbzirc_ign`

## Running the final demo

0. run `source /opt/ros/galactic/setup.sh`. If you get a message regarding another ros version set up, run the previous command again.
1. ` ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r test/simple_arm_demo.sdf"`
2. `ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=simple_demo model:=usv x:=12.75 y:=0 z:=0.3 R:=0 P:=0 Y:=1.571 arm:=mbzirc_simple_arm gripper:=mbzirc_suction_gripper`
3. `ros2 run boat_control odom`
