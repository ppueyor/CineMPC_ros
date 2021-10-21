#include <airsim_ros_pkgs/IntrinsicsCamera.h>
#include <cinempc/Constraints.h>
#include <cinempc/GetNextPersonPoses.h>
#include <cinempc/MPCIncomingState.h>
#include <cinempc/MPCResult.h>
#include <nav_msgs/Odometry.h>
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

#include <Constants.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <random>

nav_msgs::Odometry curr_odom_;

ros::Publisher intrinsics_publisher;
ros::Publisher new_state_publisher;
ros::Subscriber airsim_odom_sub_;
ros::Subscriber home_geopoint_sub_;
ros::ServiceServer local_position_goal_srvr_;
ros::ServiceServer local_position_goal_override_srvr_;
ros::ServiceServer gps_goal_srvr_;
ros::ServiceServer gps_goal_override_srvr_;

ros::Timer cinempc_calculate_new_states_timer_;