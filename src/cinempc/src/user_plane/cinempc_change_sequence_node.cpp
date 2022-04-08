#include <ros/spinner.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <user/Constants.h>

#include "ros/ros.h"

ros::Time start_time;
float current_sequence = 0, sequence = 0;

void restartSimulation(const std_msgs::Bool bool1)
{
  start_time = ros::Time::now();
  current_sequence = 0;
  std::cout << "restart" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_sequence_node");
  ros::NodeHandle n;

  ros::Publisher change_sequence_pub = n.advertise<std_msgs::Float32>("cinempc/sequence", 1000);
  ros::Rate loop_rate(5 * sim_frequency);

  ros::Subscriber restart_simulation =
	  n.subscribe<std_msgs::Bool>("cinempc/restart_simulation", 1000, restartSimulation);

  ros::Time start_time = ros::Time::now();
  float current_sequence = 0, sequence = 0;

  while (ros::ok())
  {
	ros::Duration delayed_time = ros::Time::now() - start_time;
	if (delayed_time.sec > (start_sequence_5 * sim_speed))
	{
	  current_sequence = 5;
	}
	else if (delayed_time.sec > (start_sequence_4 * sim_speed))
	{
	  current_sequence = 4;
	}
	else if (delayed_time.sec > (start_sequence_3 * sim_speed))
	{
	  current_sequence = 3;
	}
	else if (delayed_time.sec > (start_sequence_2_5 * sim_speed))
	{
	  current_sequence = 2.5;
	}
	else if (delayed_time.sec > (start_sequence_2 * sim_speed))
	{
	  current_sequence = 2;
	}
	else if (delayed_time.sec > (start_sequence_1 * sim_speed))
	{
	  current_sequence = 1;
	}
	else if (delayed_time.sec > (start_sequence_0_5 * sim_speed))
	{
	  current_sequence = 0.5;
	}
	else
	{
	  current_sequence = 0;
	}
	if (current_sequence != sequence)
	{
	  std_msgs::Float32 msg;
	  sequence = current_sequence;
	  msg.data = sequence;
	  change_sequence_pub.publish(msg);
	}
	if (current_sequence == final_sequence)
	{
	  start_time = ros::Time::now();
	  current_sequence = 0, sequence = 0;
	}
	ros::spinOnce();

	loop_rate.sleep();
  }

  return 0;
}