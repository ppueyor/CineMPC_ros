#include <airsim_ros_pkgs/GimbalAngleQuatCmd.h>
#include <airsim_ros_pkgs/IntrinsicsCamera.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <cinempc/CineMPCCommon.h>
#include <cinempc/Constraints.h>
#include <cinempc/EstimationIn.h>
#include <cinempc/GetNNextTargetPoses.h>
#include <cinempc/GetUserConstraints.h>
#include <cinempc/LowLevelControl.h>
#include <cinempc/MPCIncomingState.h>
#include <cinempc/MPCResult.h>
#include <cinempc/MPCResultPlotValues.h>
#include <cinempc/MeasurementIn.h>
#include <cinempc/MeasurementOut.h>
#include <cinempc/PlotValues.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <stdio.h>

#include <chrono>
#include <eigen3/Eigen/Core>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "ros/ros.h"
ros::Publisher set_vehicle_pose_publisher;
ros::Publisher perception_meas_publisher;
ros::Publisher restart_simulation_publisher;
ros::Publisher gimbal_rotation_publisher;
ros::Publisher new_MPC_state_publisher;
ros::Publisher low_level_control_publisher;
ros::Publisher estimation_in_publisher;

ros::ServiceClient service_take_off;

ros::Timer cinempc_calculate_new_states_timer_;

geometry_msgs::Pose drone_pose;