#include "cinempc_low_level_node.h"

using namespace std;
using namespace std::chrono;

cinempc::PlotValues plot_values;

float vel_x, vel_y, vel_z;
float focal_length = 35, focus_distance = 10000, aperture = 22;

int index_splines = -1;
bool noise = true;
float sequence = 1;
int change_sequence_index = 0;
double steps_each_dt = 40;
double interval = mpc_dt / steps_each_dt;
double freq_loop = 1 / interval;
bool stop = false, low_cost = false;

double yaw_gimbal = drone_start_yaw, pitch_gimbal = 0;

ros::Time start_log;
ros::Time start_measure;

std::stringstream logErrorFileName;

float focal_length_next_state = 35;
;

std::vector<float> times_vector, focal_length_vector, focus_distance_vector, aperture_vector, roll_vector, yaw_vector,
	pitch_vector;

void myPoseMsgToTF(const geometry_msgs::Pose& msg, tf2::Transform& bt)
{
  bt = tf2::Transform(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
					  tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
}

void lowLevelControlInCallback(const cinempc::LowLevelControl::ConstPtr& msg)
{
  focal_length_vector = msg->focal_length_vector;
  aperture_vector = msg->aperture_vector;
  pitch_vector = msg->pitch_vector;
  yaw_vector = msg->yaw_vector;
  focus_distance_vector = msg->focus_distance_vector;

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

airsim_ros_pkgs::IntrinsicsCamera getInstrinscsMsg(float focal_length_in, float focus_distance_in, float aperture_in)
{
  airsim_ros_pkgs::IntrinsicsCamera msg;
  msg.focal_length = focal_length_in;
  msg.focus_distance = focus_distance_in;
  msg.aperture = aperture_in;
  return msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_main");

  ros::NodeHandle n;

  low_level_control_subscriber = n.subscribe("cinempc/low_level_control", 1000, lowLevelControlInCallback);

  gimbal_rotation_publisher =
	  n.advertise<airsim_ros_pkgs::GimbalAngleQuatCmd>("/airsim_node/gimbal_angle_quat_cmd", 10);

  fpv_intrinsics_publisher = n.advertise<airsim_ros_pkgs::IntrinsicsCamera>("/airsim_node/drone_1/set_intrinsics", 10);
  move_on_path_publisher = n.advertise<airsim_ros_pkgs::MoveOnPath>("/airsim_node/drone_1/move_on_path", 10);

  // init camera pose
  airsim_ros_pkgs::GimbalAngleQuatCmd msg;
  geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, 0, drone_start_yaw);
  msg.orientation = q;
  gimbal_rotation_publisher.publish(msg);

  double yaw_step = 0, pitch_step = 0, focal_step = 0, focus_step = 0, ap_step;

  ros::Rate loop_rate(freq_loop);
  while (ros::ok() && !stop)
  {
	if (index_splines != -1)
	{
	  double diff_yaw, diff_pitch, diff_focal, diff_focus, diff_ap;
	  if (index_splines == 0)
	  {
		diff_focal = focal_length_vector.at(1) - focal_length_vector.at(0);
		diff_ap = aperture_vector.at(1) - aperture_vector.at(0);
		diff_yaw = yaw_vector.at(1) - yaw_vector.at(0);
		diff_pitch = pitch_vector.at(1) - pitch_vector.at(0);
		diff_focus = focus_distance_vector.at(1) - focus_distance_vector.at(0);

		yaw_step = diff_yaw / steps_each_dt;
		pitch_step = diff_pitch / steps_each_dt;
		focal_step = diff_focal / steps_each_dt;
		ap_step = diff_ap / steps_each_dt;
		focus_step = diff_focus / steps_each_dt;

		if (abs(diff_yaw) < 0.001)
		{
		  yaw_step = 0;
		}
		if (abs(diff_pitch) < 0.001)
		{
		  pitch_step = 0;
		}
		if (abs(diff_focal) < 0.5)
		{
		  focal_step = 0;
		}
	  }

	  focal_length = focal_length + focal_step;
	  yaw_gimbal = yaw_gimbal + yaw_step;
	  pitch_gimbal = pitch_gimbal + pitch_step;
	  aperture = aperture + ap_step;
	  focus_distance = focus_distance + focus_step;

	  geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, pitch_gimbal, yaw_gimbal);

	  airsim_ros_pkgs::GimbalAngleQuatCmd msg;
	  msg.orientation = q;

	  gimbal_rotation_publisher.publish(msg);
	  fpv_intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance * 100, aperture));

	  index_splines++;
	}
	loop_rate.sleep();
	ros::spinOnce();
  }
}