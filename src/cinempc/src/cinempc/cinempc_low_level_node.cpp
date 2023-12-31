#include "cinempc/cinempc_low_level_node.h"

using namespace std;

float focal_length = 35, focus_distance = 10000, aperture = 22;
int index_splines = -1;
double interval = mpc_dt / steps_each_dt_low_level;
double freq_loop = 1 / interval;
bool stop = false, low_cost = false;
double yaw_gimbal = drone_start_yaw, pitch_gimbal = 0;

std::vector<float> times_vector, focal_length_vector, focus_distance_vector, aperture_vector, roll_vector, yaw_vector,
    pitch_vector;

float interpolate(vector<float>& xData, vector<float>& yData, float x, bool extrapolate)
{
  int size = xData.size();

  int i = 0;                 // find left end of interval for interpolation
  if (x >= xData[size - 2])  // special case: beyond right end
  {
    i = size - 2;
  }
  else
  {
    while (x > xData[i + 1])
      i++;
  }
  float xL = xData[i], yL = yData[i], xR = xData[i + 1],
        yR = yData[i + 1];  // points on either side (unless beyond ends)
  if (!extrapolate)         // if beyond ends of array and not extrapolating
  {
    if (x < xL)
      yR = yL;
    if (x > xR)
      yL = yR;
  }

  float dydx = (yR - yL) / (xR - xL);  // gradient

  return yL + dydx * (x - xL);  // linear interpolation
}

void myPoseMsgToTF(const geometry_msgs::Pose& msg, tf2::Transform& bt)
{
  bt = tf2::Transform(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                      tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
}

std::vector<double> convertFloatToDoubleVector(std::vector<float> float_vector)
{
  std::vector<double> double_vec(float_vector.begin(), float_vector.end());
  return double_vec;
}

airsim_ros_pkgs::IntrinsicsCamera getInstrinscsMsg(float focal_length_in, float focus_distance_in, float aperture_in)
{
  airsim_ros_pkgs::IntrinsicsCamera msg;
  msg.focal_length = focal_length_in;
  msg.focus_distance = focus_distance_in;
  msg.aperture = aperture_in;
  return msg;
}

void restartSimulation(const std_msgs::Bool bool1)
{
  times_vector.clear();
  focal_length_vector.clear();
  focus_distance_vector.clear();
  aperture_vector.clear();
  roll_vector.clear();
  yaw_vector.clear();
  pitch_vector.clear();

  focal_length = 35, focus_distance = 10000, aperture = 22;
  index_splines = -1;
  yaw_gimbal = drone_start_yaw, pitch_gimbal = 0;

  geometry_msgs::Quaternion q = cinempc::RPY_to_quat<double>(0, pitch_gimbal, yaw_gimbal);

  airsim_ros_pkgs::GimbalAngleQuatCmd msg;
  msg.vehicle_name = vehicle_name;
  msg.camera_name = camera_name;
  msg.orientation = q;

  gimbal_rotation_publisher.publish(msg);
  fpv_intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance * 100, aperture));
  std::vector<geometry_msgs::Point> pathMPC;
  geometry_msgs::Point path_point;
  pathMPC.push_back(path_point);
  airsim_ros_pkgs::MoveOnPath msg_move_on_path;
  msg_move_on_path.vel = 0;
  msg_move_on_path.timeout = 10;
  msg_move_on_path.rads_yaw = drone_start_yaw;
  msg_move_on_path.positions = pathMPC;

  move_on_path_publisher.publish(msg_move_on_path);
}

void lowLevelControlInCallback(const cinempc::LowLevelControl::ConstPtr& msg)
{
  index_splines = -1;
  times_vector = msg->times_vector;

  focal_length_vector = msg->focal_length_vector;
  focal_length_vector.at(0) = focal_length;

  focus_distance_vector = msg->focus_distance_vector;
  focus_distance_vector.at(0) = focus_distance;

  aperture_vector = msg->aperture_vector;
  aperture_vector.at(0) = aperture;

  pitch_vector = msg->pitch_vector;
  pitch_vector.at(0) = pitch_gimbal;

  yaw_vector = msg->yaw_vector;
  yaw_vector.at(0) = yaw_gimbal;

  double distance = cinempc::calculateDistanceTo3DPoint<double>(
      msg->move_on_path_msg.positions.at(0).x, msg->move_on_path_msg.positions.at(0).y,
      msg->move_on_path_msg.positions.at(0).z, msg->move_on_path_msg.positions.at(MPC_N - 2).x,
      msg->move_on_path_msg.positions.at(MPC_N - 2).y, msg->move_on_path_msg.positions.at(MPC_N - 2).z);

  if (distance > 0.20)
  {
    move_on_path_publisher.publish(msg->move_on_path_msg);
  }

  index_splines = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_low_level");

  ros::NodeHandle n;

  low_level_control_subscriber = n.subscribe("/cinempc/low_level_control", 1000, lowLevelControlInCallback);

  gimbal_rotation_publisher =
      n.advertise<airsim_ros_pkgs::GimbalAngleQuatCmd>("/airsim_node/gimbal_angle_quat_cmd", 10);

  fpv_intrinsics_publisher = n.advertise<airsim_ros_pkgs::IntrinsicsCamera>(
      "/airsim_node/" + vehicle_name + "/" + camera_name + "/set_intrinsics", 10);
  move_on_path_publisher =
      n.advertise<airsim_ros_pkgs::MoveOnPath>("/airsim_node/" + vehicle_name + "/move_on_path", 10);

  ros::Subscriber restart_simulation =
      n.subscribe<std_msgs::Bool>("cinempc/restart_simulation", 1000, restartSimulation);

  // init camera pose
  airsim_ros_pkgs::GimbalAngleQuatCmd msg;
  geometry_msgs::Quaternion q = cinempc::RPY_to_quat<double>(0, 0, drone_start_yaw);
  msg.orientation = q;
  msg.vehicle_name = vehicle_name;
  msg.camera_name = camera_name;
  gimbal_rotation_publisher.publish(msg);

  ros::Rate loop_rate(freq_loop * sim_frequency);
  while (ros::ok() && !stop)
  {
    if (index_splines != -1 && index_splines < steps_each_dt_low_level)
    {
      focal_length = interpolate(times_vector, focal_length_vector, interval * index_splines, true);
      focus_distance = interpolate(times_vector, focus_distance_vector, interval * index_splines, true);
      aperture = interpolate(times_vector, aperture_vector, interval * index_splines, true);
      yaw_gimbal = interpolate(times_vector, yaw_vector, interval * index_splines, true);
      pitch_gimbal = interpolate(times_vector, pitch_vector, interval * index_splines, true);

      geometry_msgs::Quaternion q = cinempc::RPY_to_quat<double>(0, pitch_gimbal, yaw_gimbal);

      airsim_ros_pkgs::GimbalAngleQuatCmd msg;
      msg.vehicle_name = vehicle_name;
      msg.camera_name = camera_name;
      msg.orientation = q;

      gimbal_rotation_publisher.publish(msg);
      fpv_intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance * 100, aperture));

      index_splines++;
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}