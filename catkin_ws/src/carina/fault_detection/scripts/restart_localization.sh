#!/bin/bash

rosnode kill ekf_node imu_broadcaster_node ins_to_utm_pose_node stereo_odometry &
#sleep 2 &
echo "[Restarting localization] running..."&
roslaunch lrm_localization_stack null_sample_stack.launch #&
#sleep 1
exit 0


