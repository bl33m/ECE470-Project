# ECE470-Project

To run the simulator:

In seperate terminals run each of the following commands

1. `ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r coast.sdf"`
2. `ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=coast model:=usv x:=-1462 y:=-16.5 z:=0.3 R:=0 P:=0 Y:=0  arm:=mbzirc_oberon7_arm gripper:=mbzirc_oberon7_gripper`
3. `ros2 run joy joy_node`
4. `ros2 run boat_control joyctl`
