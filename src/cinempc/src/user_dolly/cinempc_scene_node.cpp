#include <geometry_msgs/PoseArray.h>
#include <ros/spinner.h>
#include <std_msgs/Float32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ros/ros.h"
#include "user/Constants.h"

float sequence = 1;
std::vector<double> move_target_x_per_step, move_target_y_per_step, move_target_z_per_step, yaw_target;
std::vector<ros::Publisher> move_target_pub_vector, speed_target_pub;

std::vector<geometry_msgs::PoseStamped> initial_state_target_vector;

void changeSeqCallback(const std_msgs::Float32::ConstPtr &msg)
{
  sequence = msg->data;
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
	initial_state_target_vector.push_back(*initial_state_ptr);

	move_target_pub_vector.push_back(
		n.advertise<geometry_msgs::Pose>("airsim_node/" + targets_names.at(i) + "/set_pose", 1000));

	speed_target_pub.push_back(
		n.advertise<geometry_msgs::Point>("airsim_node/" + targets_names.at(i) + "/current_speed", 1000));

	move_target_x_per_step.push_back(0);
	move_target_y_per_step.push_back(0);
	move_target_z_per_step.push_back(0);
	yaw_target.push_back(0);
  }

  ros::Subscriber change_sequence_sub = n.subscribe("cinempc/sequence", 1000, changeSeqCallback);

  double frequency = 400 * sim_frequency;
  ros::Rate loop_rate(frequency);

  double current_speed_x = 0, current_speed_y = 0, current_speed_z = 0;

  double current_pitch = 0, current_yaw = 0;

  double steps_velocity = 0;
  double max_vel = 0;

  while (ros::ok())
  {
	if (sequence == 1)
	{
	  if (current_speed_y > -max_vel)
	  {
		current_speed_y -= steps_velocity;
	  }
	}

	move_target_x_per_step.at(0) = current_speed_x;	 
	move_target_y_per_step.at(0) = current_speed_y;	
	move_target_z_per_step.at(0) = current_speed_z;	

	for (int i = 0; i < targets_names.size(); i++)
	{
	  initial_state_target_vector.at(i).pose.position.x =
		  initial_state_target_vector.at(i).pose.position.x + move_target_x_per_step.at(i);
	  initial_state_target_vector.at(i).pose.position.y =
		  initial_state_target_vector.at(i).pose.position.y + move_target_y_per_step.at(i);
	  initial_state_target_vector.at(i).pose.position.z =
		  initial_state_target_vector.at(i).pose.position.z + move_target_z_per_step.at(i);

	  tf2::Quaternion quaternion_tf;
	  quaternion_tf.setRPY(current_pitch, 0, current_yaw);
	  quaternion_tf.normalize();

	  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quaternion_tf);

	  initial_state_target_vector.at(i).pose.orientation = quat_msg;

	  geometry_msgs::Point velocity_target;
	  velocity_target.x = calculateSpeed(current_speed_x, frequency);
	  velocity_target.y = calculateSpeed(current_speed_y, frequency);
	  velocity_target.z = calculateSpeed(current_speed_z, frequency);
	  if (publish_topics_gt)
	  {
		move_target_pub_vector.at(i).publish(initial_state_target_vector.at(i).pose);
		speed_target_pub.at(i).publish(velocity_target);
	  }
	}
	ros::spinOnce();

	loop_rate.sleep();
  }

  return 0;
}