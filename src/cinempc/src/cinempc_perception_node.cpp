#include "QuaternionConverters.h"
#include "cinempc_perception_node.h"

using namespace cv;
using namespace std;

Eigen::Matrix<double, 3, 3> quatToRMatrix(geometry_msgs::Quaternion q)
{
  // double roll, pitch, yaw;
  // tf2::Quaternion quat_tf;
  // tf2::fromMsg(q, quat_tf);
  // Eigen::Matrix<double, 3, 3> mat_res;
  // tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  // return RPYtoRMatrix(roll, pitch, yaw);
}

void newImageReceivedCallback(const cinempc::PerceptionMsg& msg)
{
  int personsFound = 0;
  Mat m_rgb = cv::imdecode(msg.rgb.data, -1);
  DarkHelp::PredictionResults results = darkhelp.predict(m_rgb);
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
  if (personsFound > 0)
  {
    Rect rect1 = result.rect;
    float target_u = rect1.x + (rect1.width / 2);
    float target_v = rect1.y + (rect1.height / 8);
    float target_v_center = rect1.y + (rect1.height / 2);

    int pixel = (target_v_center * msg.rgb.width) + target_u;
    float depth = msg.rgb.data[pixel];

    geometry_msgs::Point point3D =
        cinempc::readPositionImageToWorld<double>(msg.drone_state.intrinsics.focal_length, target_u, target_v, depth,
                                                  msg.drone_state.drone_pose.orientation);  // return meters
    float boy_x_cv = point3D.x;
    float boy_y_cv = point3D.y;
    float boy_z_cv = -point3D.z;
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

  ros::Subscriber image_received_sub = n.subscribe("cinempc/perception", 1000, newImageReceivedCallback);

  ros::spin();

  return 0;
}
