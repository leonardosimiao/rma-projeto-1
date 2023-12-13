#!/bin/bash
mkdir -p /home/$USER/z_bags/$(date +%Y_%m_%d__%H_%M_%S) && cd $_
rosbag record /cmd_vel /odom /scan /base_scan /slam_cloud /sonar_back /sonar_front /tf /tf_static /slam_out_pose /pose2d /move_base /diagnostics /move_base_simple/goal /trajectory /robot/cmd_vel /robot/cmd_vel_teleop /clicked_point /control/message /control/move_base /poseupdate /robot/init_pose /robot/message /syscommand /initialpose /laser_status /move_base/TebLocalPlannerROS/global_plan /move_base/TebLocalPlannerROS/local_plan /move_base/TebLocalPlannerROS/teb_markers /move_base/TebLocalPlannerROS/teb_markers_array /move_base/TebLocalPlannerROS/teb_poses /particlecloud /sensor_pointcloud_node/pointcloud /urg_node10lx/parameter_descriptions /urg_node10lx/parameter_updates -o mob_rob