#include <CineMPCCommon.h>
#include <airsim_ros_pkgs/GimbalAngleQuatCmd.h>
#include <airsim_ros_pkgs/IntrinsicsCamera.h>
#include <airsim_ros_pkgs/MoveOnPath.h>
#include <cinempc/LowLevelControl.h>
#include <spline.h>

#include <vector>

#include "ros/ros.h"

ros::Publisher fpv_intrinsics_publisher;
ros::Publisher gimbal_rotation_publisher;
ros::Publisher move_on_path_publisher;

ros::Subscriber low_level_control_subscriber;