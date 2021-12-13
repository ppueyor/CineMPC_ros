#include "main_node.h"

bool stop = false;

ros::Time start_log;

std::stringstream logErrorFileName;
std::ofstream errorFile;

void stopReceivedCallback(const std_msgs::Bool& msg)
{
  stop = msg.data;
}

void newPoseReceived(const ros::TimerEvent& e, ros::NodeHandle n)
{
  ros::Time end_log = ros::Time::now();
  ros::Duration diff = end_log - start_log;

  int a_received, b_received, c_received;

  errorFile << diff.toNSec() / 1000000 << "," << a_received << "," << b_received << "," << c_received << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "main");

  auto const now = std::chrono::system_clock::now();
  auto const in_time_t = std::chrono::system_clock::to_time_t(now);
  logErrorFileName << "/home/pablo/Desktop/AirSim_update/AirSim_ros/ros/src/cinempc/logs/error_pos_"
                   << std::put_time(std::localtime(&in_time_t), "%d_%m_%Y-%H_%M_%S") << ".csv";
  errorFile.open(logErrorFileName.str());  // pitch,roll,yaw for every time stamp
  errorFile << "Time"
            << ","
            << "a"
            << ","
            << "b"
            << ","
            << "c" << std::endl;

  ros::NodeHandle n;

  start_log = ros::Time::now();
  ros::Subscriber new_rgb_received = n.subscribe("/pose", 1000, newPoseReceived);

  ros::Subscriber stop_signal_received = n.subscribe("/stop", 1000, stopReceivedCallback);

  while (ros::ok() && !stop)
  {
    ros::spinOnce();
  }
  errorFile.close();
  return 0;
}
