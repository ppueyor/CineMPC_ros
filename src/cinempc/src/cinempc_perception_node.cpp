#include "QuaternionConverters.h"
#include "cinempc_perception_node.h"

using namespace cv;
using namespace std;

void newImageReceivedCallback(const cinempc::PerceptionMsg& msg)
{
  int personsFound = 0;
  auto data = msg.rgb.data;
  cv_bridge::CvImagePtr rgb_cv_ptr, depth_cv_ptr;
  try
  {
    rgb_cv_ptr = cv_bridge::toCvCopy(msg.rgb, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  try
  {
    depth_cv_ptr = cv_bridge::toCvCopy(msg.depth, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  DarkHelp::PredictionResults results = darkhelp.predict(rgb_cv_ptr->image);
  DarkHelp::PredictionResult result;
  if (results.size() != 0)
  {
    for (DarkHelp::PredictionResult result_vector : results)
    {
      if (result_vector.name.find("person") != std::string::npos && result_vector.best_probability > 0.80)
      {
        result = result_vector;
        personsFound++;
      }
    }
  }
  cv::Mat output = darkhelp.annotate();
  int a = 0;
  cv::imwrite("/home/pablo/Desktop/AirSim_update/AirSim_ros/ros/src/cinempc/images/b.jpg", rgb_cv_ptr->image);
  cv::imwrite("/home/pablo/Desktop/AirSim_update/AirSim_ros/ros/src/cinempc/images/a.jpg", output);

  if (personsFound > 0)
  {
    Rect rect1 = result.rect;
    float target_u = rect1.x + (rect1.width / 2);
    float target_v = rect1.y + (rect1.height / 8);
    float target_v_center = rect1.y + (rect1.height / 2);

    int pixel = (target_v_center * msg.rgb.width) + target_u;

    float depth_target = depth_cv_ptr->image.at<float>(target_v_center, target_u) * 100000;  // convert to mms

    geometry_msgs::Point point3D = cinempc::readPositionImageToWorld<double>(
        msg.drone_state.intrinsics.focal_length, target_u, target_v, depth_target,
        msg.drone_state.drone_pose.orientation);  // return meters
    float target_x_cv = point3D.x;
    float target_y_cv = point3D.y;
    float target_z_cv = -point3D.z;

    geometry_msgs::Pose p;
    p.position = point3D;

    perception_result_publisher.publish(p);

    // std::cout << "yaw1:" << target_x_cv << "u: " << target_y_cv << "v:" << target_z_cv << std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_perception");

  const std::string config_file = "/home/pablo/Downloads/darknet/cfg/yolov4.cfg";
  const std::string weights_file = "/home/pablo/Downloads/darknet/yolov4.weights";
  const std::string names_file = "/home/pablo/Downloads/darknet/cfg/coco.names";

  darkhelp.init(config_file, weights_file, names_file);
  darkhelp.threshold = 0.4;
  darkhelp.include_all_names = true;
  darkhelp.names_include_percentage = true;
  darkhelp.annotation_include_duration = false;
  darkhelp.annotation_include_timestamp = false;
  darkhelp.sort_predictions = DarkHelp::ESort::kDescending;

  ros::NodeHandle n;

  ros::Subscriber image_received_sub = n.subscribe("/cinempc/perception", 1000, newImageReceivedCallback);
  perception_result_publisher = n.advertise<geometry_msgs::Pose>("/cinempc/target_pose_perception", 10);

  ros::spin();

  return 0;
}
