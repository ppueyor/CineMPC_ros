#include <geometry_msgs/PoseArray.h>
#include <ros/spinner.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ros/ros.h"
#include "user/Constants.h"

float sequence = 1;
std::vector<double> move_target_x_per_step, move_target_y_per_step, move_target_z_per_step,
	yaw_target;	 // for each target
std::vector<ros::Publisher> move_target_pub_vector, speed_target_pub;

std::vector<geometry_msgs::PoseStamped> state_target_vector;
std::vector<geometry_msgs::PoseStamped> initial_state_target_vector;

geometry_msgs::Pose initial_pose_plane;
bool initial_pose = false;

void changeSeqCallback(const std_msgs::Float32::ConstPtr &msg)
{
  sequence = msg->data;
}

void restartSimulation(const std_msgs::Bool bool1)
{
  for (int i = 0; i < 1; i++)
  {
	move_target_pub_vector.at(i).publish(initial_state_target_vector.at(i).pose);
	sequence = 0;

	state_target_vector.clear();
	state_target_vector.push_back(initial_state_target_vector.at(i));
  }
}

double calculateSpeed(double speed_freq, double freq)
{
  double time = 1 / freq;
  return (speed_freq / time) * sim_speed;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scene_node");
  ros::NodeHandle n;

  for (int i = 0; i < 1; i++)
  {
	boost::shared_ptr<geometry_msgs::PoseStamped const> initial_state_ptr =
		ros::topic::waitForMessage<geometry_msgs::PoseStamped>("airsim_node/" + targets_names.at(i) + "/get_pose");
	if (!initial_pose)
	{
	  initial_state_target_vector.push_back(*initial_state_ptr);
	  initial_pose = true;
	}
	state_target_vector.push_back(*initial_state_ptr);

	move_target_pub_vector.push_back(
		n.advertise<geometry_msgs::Pose>("airsim_node/" + targets_names.at(i) + "/set_pose", 1000));

	speed_target_pub.push_back(
		n.advertise<geometry_msgs::Point>("airsim_node/" + targets_names.at(i) + "/current_speed", 1000));

	move_target_x_per_step.push_back(0);  // - camera_adjustement;
	move_target_y_per_step.push_back(0);
	move_target_z_per_step.push_back(0);
	yaw_target.push_back(0);
  }
  ros::Subscriber change_sequence_sub = n.subscribe("cinempc/sequence", 1000, changeSeqCallback);

  ros::Subscriber restart_simulation =
	  n.subscribe<std_msgs::Bool>("cinempc/restart_simulation", 1000, restartSimulation);

  double frequency = 400 * sim_frequency;
  ros::Rate loop_rate(frequency);
  double current_speed_x = 0, current_speed_y = 0, current_speed_z = 0;
  double current_pitch = 0, current_yaw = 0;
  double steps_velocity = 0.000004;
  double max_vel = 0.025;

  double steps_z = 0.000005;
  double steps_rot = 0.00025;
  while (ros::ok())
  {
	if (sequence == 0 || sequence == 0.5)
	{
	  current_speed_x = 0, current_speed_y = 0, current_speed_z = 0;
	  current_pitch = 0, current_yaw = 0;
	}
	else if (sequence == 1)
	{
	  if (current_speed_y > -max_vel)
	  {
		current_speed_y -= steps_velocity;
	  }
	}
	else if (sequence == 2)
	{
	  if (current_speed_z > -0.005)
	  {
		current_speed_z -= steps_z;
	  }

	  if (current_pitch < 0.2)
	  {
		current_pitch += steps_rot;
	  }
	}
	else if (sequence == 2.5)
	{
	  if (current_speed_z < 0)
	  {
		current_speed_z += steps_z;
	  }

	  if (current_pitch > 0)
	  {
		current_pitch -= steps_rot;
	  }
	}
	else if (sequence == 3)
	{
	  {
		if (current_speed_x < max_vel)
		{
		  current_speed_x += steps_velocity;
		}

		if (current_speed_y < 0)
		{
		  current_speed_y += steps_velocity;
		}
		if (current_yaw < PI / 2)
		{
		  current_yaw += steps_rot;
		}

		if (current_pitch > 0)
		{
		  current_pitch -= steps_rot;
		}
		if (current_speed_z < 0)
		{
		  current_speed_z += steps_z;
		}
	  }
	}
	else if (sequence == 4)
	{
	  {
	  }
	}
	move_target_x_per_step.at(0) = current_speed_x;
	move_target_y_per_step.at(0) = current_speed_y;
	move_target_z_per_step.at(0) = current_speed_z;

	for (int i = 0; i < 1; i++)
	{
	  state_target_vector.at(i).pose.position.x =
		  state_target_vector.at(i).pose.position.x + move_target_x_per_step.at(i);
	  state_target_vector.at(i).pose.position.y =
		  state_target_vector.at(i).pose.position.y + move_target_y_per_step.at(i);
	  state_target_vector.at(i).pose.position.z =
		  state_target_vector.at(i).pose.position.z + move_target_z_per_step.at(i);

	  tf2::Quaternion quaternion_tf;
	  quaternion_tf.setRPY(current_pitch, 0, current_yaw);
	  quaternion_tf.normalize();

	  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quaternion_tf);

	  state_target_vector.at(i).pose.orientation = quat_msg;

	  geometry_msgs::Point velocity_target;
	  velocity_target.x = calculateSpeed(current_speed_x, frequency);
	  velocity_target.y = calculateSpeed(current_speed_y, frequency);
	  velocity_target.z = calculateSpeed(current_speed_z, frequency);

	  move_target_pub_vector.at(i).publish(state_target_vector.at(i).pose);

	  if (publish_topics_gt)
	  {
		speed_target_pub.at(i).publish(velocity_target);
	  }
	}

	ros::spinOnce();

	loop_rate.sleep();
  } 

  return 0;
}