#include <cinempc/GetNextPersonPoses.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/spinner.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Constants.h"
#include "common/AirSimSettings.hpp"
#include "ros/ros.h"

float sequence = 1;
double move_person_step_dt = 10;
std::vector<double> move_target_x_per_step, move_target_y_per_step, move_target_z_per_step, yaw_target;

geometry_msgs::PoseStamped initial_state;

void changeSeqCallback(const std_msgs::Float32::ConstPtr &msg)
{
  sequence = msg->data;
}

bool getNStepsTargetService(cinempc::GetNextPersonPoses::Request &req, cinempc::GetNextPersonPoses::Response &res,
							int index)
{
  std::vector<geometry_msgs::Pose> array_of_poses;

  geometry_msgs::Pose initial_pose = req.current_pose;
  for (int i = 0; i < MPC_N * 2; i++)
  {
	geometry_msgs::Pose current_pose;
	if (i < MPC_N)
	{
	  current_pose.position.x =
		  initial_pose.position.x;	// + move_target_x_per_step.at(index) * move_person_step_dt * i;
	  current_pose.position.y =
		  initial_pose.position.y;	// + move_target_y_per_step.at(index) * move_person_step_dt * i;
	  current_pose.position.z = initial_pose.position.z;
	}
	else
	{
	  if (sequence == 1 || sequence == 3)
	  {
		current_pose.position.x =
			initial_pose.position.x;  // + move_target_x_per_step.at(index) * move_person_step_dt * (i - MPC_N);
		current_pose.position.y =
			initial_pose.position.y;  // + move_target_y_per_step.at(index) * move_person_step_dt * (i - MPC_N);
		current_pose.position.z = initial_pose.position.z + 0.5;
	  }
	  else
	  {
		current_pose.position.x =
			initial_pose.position.x + move_target_x_per_step.at(index) * move_person_step_dt * (i - MPC_N);
		current_pose.position.y =
			initial_pose.position.y + move_target_y_per_step.at(index) * move_person_step_dt * (i - MPC_N);
		current_pose.position.z = initial_pose.position.z + 1.2;
	  }
	}
	current_pose.orientation = initial_pose.orientation;
	array_of_poses.push_back(current_pose);
  }
  res.pose_array.poses = array_of_poses;
  return true;
}

tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr &airlib_quat)
{
  return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_person_node");
  ros::NodeHandle n;

  boost::shared_ptr<geometry_msgs::PoseStamped const> initial_state_ptr =
	  ros::topic::waitForMessage<geometry_msgs::PoseStamped>("airsim_node/Person1/get_pose");
  initial_state = *initial_state_ptr;

  std::vector<ros::ServiceServer> mpc_n_target_steps_service;
  for (int i = 0; i < targets_names.size(); i++)
  {
	mpc_n_target_steps_service.push_back(
		n.advertiseService<cinempc::GetNextPersonPoses::Request, cinempc::GetNextPersonPoses::Response>(
			"cinempc/" + targets_names.at(i) + "/get_next_poses", boost::bind(&getNStepsTargetService, _1, _2, i)));
	move_target_x_per_step.push_back(0);  // - camera_adjustement;
	move_target_y_per_step.push_back(0);
	move_target_z_per_step.push_back(0);
	yaw_target.push_back(0);
  }

  ros::Publisher move_person_pub = n.advertise<geometry_msgs::Pose>("airsim_node/Person1/set_pose", 1000);

  ros::Subscriber change_sequence_sub = n.subscribe("cinempc/sequence", 1000, changeSeqCallback);

  ros::Rate loop_rate(33);

  while (ros::ok())
  {
	if (sequence == 1 || sequence == 2)
	{
	  move_target_x_per_step.at(0) = 0;	 // 0.005;	 // - camera_adjustement;
	  move_target_y_per_step.at(0) = 0.015;
	  move_target_z_per_step.at(0) = 0;	 //-0.005;
	}
	else if (sequence == 2.5 || sequence == 3)
	{
	  move_target_x_per_step.at(0) = 0.02;
	  move_target_y_per_step.at(0) = 0;
	  if (yaw_target.at(0) > -M_PI / 2)
	  {
		yaw_target.at(0) = yaw_target.at(0) - 0.01;
		move_target_y_per_step.at(0) = 0.02;
	  }
	}

	initial_state.pose.position.x = initial_state.pose.position.x + move_target_x_per_step.at(0);

	initial_state.pose.position.y = initial_state.pose.position.y + move_target_y_per_step.at(0);

	initial_state.pose.position.z = initial_state.pose.position.z + move_target_z_per_step.at(0);

	tf2::Quaternion quaternion_tf;
	quaternion_tf.setRPY(0, 0, yaw_target.at(0));
	quaternion_tf.normalize();

	geometry_msgs::Quaternion quat_msg = tf2::toMsg(quaternion_tf);

	initial_state.pose.orientation = quat_msg;

	move_person_pub.publish(initial_state.pose);

	ros::spinOnce();

	loop_rate.sleep();
  }

  return 0;
}
