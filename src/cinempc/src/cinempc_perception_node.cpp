#include "QuaternionConverters.h"
#include "cinempc_perception_node.h"

using namespace cv;
using namespace std;
int i = 0;
// calculates an average depth from the square sourronding by width/heigth/3 the center of the bounding box
float calculateDepth(cv_bridge::CvImagePtr depth_cv_ptr, Rect bounding_box)
{
  float target_u_center = bounding_box.x + (bounding_box.width / 2);
  float target_v_center = bounding_box.y + (bounding_box.height / 2);

  float center_depth = depth_cv_ptr->image.at<float>(target_v_center, target_u_center);  // convert to mms

  float bounding_u_third = bounding_box.width / 3;
  float bounding_v_third = bounding_box.height / 3;
  float min_third = min(bounding_u_third, bounding_v_third);

  float acc_depth = 0, depths = 0;

  float depth_u_start = target_u_center - min_third;
  float depth_u_end = target_u_center + min_third;
  float depth_v_start = target_v_center - min_third;
  float depth_v_end = target_v_center + min_third;

  for (int i = depth_u_start; i <= depth_u_end; i++)
  {
    for (int j = depth_v_start; j <= depth_v_end; j++)
    {
      float current_depth = depth_cv_ptr->image.at<float>(j, i);

      // it is not in the target bounding box
      if (current_depth < center_depth + center_depth / 5 && current_depth > center_depth - center_depth / 5)
      {
        acc_depth += depth_cv_ptr->image.at<float>(j, i);
        depths++;
      }
    }
  }
  float avg_depth = acc_depth / depths;
  return avg_depth * 100000;
}

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
  // cv::imwrite("/home/pablo/Desktop/AirSim_update/AirSim_ros/ros/src/cinempc/images/a" + to_string(index_pic) +
  // ".jpg",
  //             output);

  if (personsFound > 0)
  {
    Rect rect1 = result.rect;
    float target_u_center = rect1.x + (rect1.width / 2);
    float target_v_top = rect1.y;  // + (rect1.height);
    float target_v_center = rect1.y + (rect1.height / 2);

    int pixel = (target_v_center * msg.rgb.width) + target_u_center;

    // float depth_target = calculateDepth(depth_cv_ptr, rect1);  // convert to mms
    float depth_target = depth_cv_ptr->image.at<float>(target_v_center, target_u_center) * 100000;  // convert to mms
    geometry_msgs::Quaternion wRt = cinempc::RPYToQuat<double>(0, 0, 0);

    geometry_msgs::Pose drone_pose_top = cinempc::drone_relative_position_from_image<double>(
        msg.drone_state.intrinsics.focal_length, target_u_center, target_v_top, depth_target,
        msg.drone_state.drone_pose.orientation, wRt);

    geometry_msgs::Pose drone_pose_head = drone_pose_top;
    drone_pose_head.position.z = drone_pose_head.position.z + 0.2;

    // we calculate the position of the head and then the rest of the body
    geometry_msgs::Pose head_pose_hips;
    head_pose_hips.orientation = cinempc::RPYToQuat<double>(0, 0, 0);
    head_pose_hips.position.z = 0;
    head_pose_hips.position.z = 0;
    head_pose_hips.position.z = 0.7;

    geometry_msgs::Pose head_pose_feet;
    head_pose_feet.orientation = cinempc::RPYToQuat<double>(0, 0, 0);
    head_pose_feet.position.z = 0;
    head_pose_feet.position.z = 0;
    head_pose_feet.position.z = 1.66;

    geometry_msgs::Pose drone_pose_hips =
        cinempc::calculate_relative_poses_drone_targets<double>(drone_pose_head, head_pose_hips);
    geometry_msgs::Pose drone_pose_feet =
        cinempc::calculate_relative_poses_drone_targets<double>(drone_pose_head, head_pose_feet);

    cinempc::PersonStatePerception person_msg;
    person_msg.pose_head = drone_pose_head;
    person_msg.pose_hips = drone_pose_hips;
    person_msg.pose_feet = drone_pose_feet;

    // TODO: SAME POSE FOR BOTH TARGETS
    for (int i = 0; i < targets_names.size(); i++)
    {
      person_msg.target_name = targets_names.at(i);
      perception_result_publishers.at(i).publish(person_msg);
    }
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
  darkhelp.config.threshold = 0.4;
  darkhelp.config.include_all_names = true;
  darkhelp.config.names_include_percentage = true;
  darkhelp.config.annotation_include_duration = false;
  darkhelp.config.annotation_include_timestamp = false;
  darkhelp.config.sort_predictions = DarkHelp::ESort::kDescending;

  ros::NodeHandle n;

  ros::Subscriber image_received_sub = n.subscribe("/cinempc/perception_in", 1000, newImageReceivedCallback);

  for (int i = 0; i < targets_names.size(); i++)
  {
    perception_result_publishers.push_back(n.advertise<cinempc::PersonStatePerception>(
        "/cinempc/" + targets_names.at(i) + "/target_state_perception", 10));
  }

  ros::spin();

  return 0;
}
