#include <cinempc/PerceptionMsg.h>
#include <cinempc/PersonStatePerception.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <DarkHelp.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "ros/ros.h"

DarkHelp::NN darkhelp;
std::vector<ros::Publisher> perception_result_publishers = {};
