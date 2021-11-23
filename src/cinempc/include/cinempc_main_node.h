#include <airsim_ros_pkgs/GimbalAngleQuatCmd.h>
#include <airsim_ros_pkgs/IntrinsicsCamera.h>
#include <airsim_ros_pkgs/MoveOnPath.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <cinempc/Constraints.h>
#include <cinempc/GetNextPersonPoses.h>
#include <cinempc/GetUserConstraints.h>
#include <cinempc/MPCIncomingState.h>
#include <cinempc/MPCResult.h>
#include <cinempc/PerceptionMsg.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <stdio.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "SimpleKalmanFilter.h"
#include "opencv4/opencv2/opencv.hpp"
#include "ros/ros.h"
#include "spline.h"
//#include <matplot/matplot.h>
#include <QuaternionConverters.h>
#include <cinempc/PersonStateMPC.h>
#include <cinempc/PersonStatePerception.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <kalman.hpp>
#include <random>

ros::Publisher fpv_intrinsics_publisher;
ros::Publisher perception_publisher;
ros::Publisher gimbal_rotation_publisher;
ros::Publisher new_MPC_state_publisher;

ros::ServiceClient service_move_on_path;

ros::Timer cinempc_calculate_new_states_timer_;

geometry_msgs::Pose drone_pose;