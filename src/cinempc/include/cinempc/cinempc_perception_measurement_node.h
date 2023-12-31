#include <cinempc/CineMPCCommon.h>
#include <cinempc/MeasurementIn.h>
#include <cinempc/MeasurementOut.h>
#include <cinempc/TargetState.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <random>

#include <DarkHelp.hpp>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "ros/ros.h"

using namespace cv;
using namespace std;

DarkHelp::NN darkhelp;
ros::Publisher perception_result_publisher;
