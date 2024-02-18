#!/bin/bash
rostopic pub -1 /carina/vehicle/shutdown std_msgs/Bool "data: true"
rosnode kill robot_state_publisher joint_state_publisher stereo_odometry &
#sleep 10 &
#roscore &
sleep 7 &
echo "[Execute Carina] running..."&
roslaunch carina_master carina.launch &
#roslaunch carina_master carina_dataset_planner.launch &
#roslaunch carina_master carinaDisparityDataset.launch &
sleep 2
exit 0
