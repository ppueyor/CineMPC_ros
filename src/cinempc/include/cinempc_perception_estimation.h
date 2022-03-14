#include <CineMPCCommon.h>
#include <cinempc/EstimationIn.h>
#include <cinempc/GetNNextTargetPoses.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <kalman.hpp>
#include <vector>

#include "ros/ros.h"

ros::Publisher gimbal_rotation_publisher;
ros::Publisher move_on_path_publisher;

ros::Subscriber KF_subscriber;