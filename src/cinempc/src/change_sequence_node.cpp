#include <ros/spinner.h>
#include <std_msgs/Float32.h>

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_sequence_node");
  ros::NodeHandle n;

  ros::Publisher change_sequence_pub = n.advertise<std_msgs::Float32>("cinempc/sequence", 1000);
  ros::Rate loop_rate(5);

  ros::Time start_time = ros::Time::now();
  float current_sequence = 0, sequence = 0;

  while (ros::ok())
  {
	ros::Duration delayed_time = ros::Time::now() - start_time;

	if (delayed_time.sec > 15)
	{
	  current_sequence = 3;
	}
	else if (delayed_time.sec > 5)
	{
	  current_sequence = 2;
	}
	else
	{
	  current_sequence = 1;
	}
	if (current_sequence != sequence)
	{
	  std_msgs::Float32 msg;
	  sequence = current_sequence;
	  msg.data = sequence;
	  change_sequence_pub.publish(msg);
	}

	ros::spinOnce();

	loop_rate.sleep();
  }

  return 0;
}
