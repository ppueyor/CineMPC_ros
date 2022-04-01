#include "cinempc/cinempc_perception_measurement_node.h"

ros::Time start_log;
int images_received = 0;
std::stringstream folder_name, name_depth, name_rgb, name_perception;

// calculates an average depth from the square sourronding by width/heigth/3 the center of the bounding box
float calculateAverageDepth(cv_bridge::CvImagePtr depth_cv_ptr, Rect bounding_box)
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

// calculates a median depth the closest pixels of every row
float calculateMedianDepth(cv_bridge::CvImagePtr depth_cv_ptr, Rect bounding_box)
{
  std::vector<float> closest_points_per_row = {};
  for (int v = bounding_box.y; v < bounding_box.y + bounding_box.height; v++)
  {
    float closest_depth_row = depth_cv_ptr->image.at<float>(0, 0);
    for (int u = bounding_box.x; u < bounding_box.x + bounding_box.width; u++)
    {
      float current_depth = depth_cv_ptr->image.at<float>(v, u);
      if (current_depth < closest_depth_row)
      {
        closest_depth_row = current_depth;
      }
    }
    closest_points_per_row.push_back(closest_depth_row);
  }
  sort(closest_points_per_row.begin(), closest_points_per_row.end());
  int median = closest_points_per_row.size() / 2;
  return closest_points_per_row.at(median) * 100000;
}

cinempc::TargetState extractTargetStateFromImg(DarkHelp::PredictionResult result, cv_bridge::CvImagePtr depth_cv_ptr,
                                               const cinempc::MeasurementIn& msg_in)
{
  Rect bb_target = result.rect;
  float target_u_center = bb_target.x + (bb_target.width / 2);
  float target_v_top = bb_target.y;
  float target_v_center = bb_target.y + (bb_target.height / 2);

  float depth_target = calculateMedianDepth(depth_cv_ptr, bb_target);  // convert to mms
  geometry_msgs::Quaternion wRt = cinempc::RPY_to_quat<double>(0, 0, 0);

  geometry_msgs::Pose relative_pose_top = cinempc::drone_target_relative_position_from_image<double>(
      msg_in.drone_state.intrinsics.focal_length, target_u_center, target_v_top, depth_target,
      msg_in.drone_state.drone_pose.orientation, wRt);

  cinempc::TargetState target_state;
  target_state.pose_top = relative_pose_top;
  target_state.target_name = result.name;

  return target_state;
}

void newImageReceivedCallback(const cinempc::MeasurementIn& msg)
{
  int targetsFound = 0;
  std::vector<cinempc::TargetState> targets_state;
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
  if (results.size() != 0)
  {
    for (DarkHelp::PredictionResult result_vector : results)
    {
      for (string target_class : targets_classes)
      {
        if (result_vector.name.find(target_class) != std::string::npos && result_vector.best_probability > 0.8)
        {
          targets_state.push_back(extractTargetStateFromImg(result_vector, depth_cv_ptr, msg));
          targetsFound++;
        }
      }
    }
  }

  // avoids saving too many images
  if (images_received % 2 == 0 && save_imgs)
  {
    cv::Mat output = darkhelp.annotate();

    ros::Time end_log = ros::Time::now();
    ros::Duration diff = end_log - start_log;
    int time_mss = diff.toNSec() / (1000000);

    std::stringstream depth_name, rgb_name, perception_name;

    depth_name << folder_name.str() << time_mss << "_depth"
               << ".jpg";
    rgb_name << folder_name.str() << time_mss << "_rgb"
             << ".jpg";
    perception_name << folder_name.str() << time_mss << "_perception"
                    << ".jpg";

    cv::imwrite(rgb_name.str(), rgb_cv_ptr->image);
    cv::imwrite(perception_name.str(), output);
  }
  images_received++;
  cinempc::MeasurementOut perception_out_msg;
  perception_out_msg.targets_found = targetsFound;
  perception_out_msg.drone_state.drone_pose = msg.drone_state.drone_pose;
  perception_out_msg.targets_state = targets_state;

  perception_result_publisher.publish(perception_out_msg);
}

int main(int argc, char** argv)
{
  if (use_perception)
  {
    ros::init(argc, argv, "cinempc_perception_measurement");

    darkhelp.init(config_file_yolo, weights_file_yolo, names_file_yolo);
    darkhelp.config.threshold = 0.4;
    darkhelp.config.include_all_names = true;
    darkhelp.config.names_include_percentage = true;
    darkhelp.config.annotation_include_duration = false;
    darkhelp.config.annotation_include_timestamp = false;
    darkhelp.config.sort_predictions = DarkHelp::ESort::kDescending;

    ros::NodeHandle n;
    start_log = ros::Time::now();
    auto const now = std::chrono::system_clock::now();
    auto const in_time_t = std::chrono::system_clock::to_time_t(now);

    folder_name << project_folder << "images/" << targets_names.at(0) << "/"
                << std::put_time(std::localtime(&in_time_t), "%d_%m_%Y-%H_%M_%S") << "/";
    common_utils::FileSystem::createDirectory(folder_name.str());
    ros::Subscriber image_received_sub =
        n.subscribe("/cinempc/perception_measurement_in", 1000, newImageReceivedCallback);

    perception_result_publisher = n.advertise<cinempc::MeasurementOut>("/cinempc/perception_measurement_out", 10);

    ros::spin();
  }

  return 0;
}