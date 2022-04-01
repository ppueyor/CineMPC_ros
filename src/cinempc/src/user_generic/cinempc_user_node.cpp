
#include <cinempc/CineMPCCommon.h>
#include <cinempc/GetUserConstraints.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include "ros/ros.h"

bool getConstraints(cinempc::GetUserConstraints::Request &req, cinempc::GetUserConstraints::Response &res)
{
  cinempc::Constraints c;

  //init
  for (int i = 0; i < req.targets_relative.size(); i++)
  {
    geometry_msgs::Point p;
    cinempc::WeightsTargets w;
    c.targets_im_top_star.push_back(p);
    c.targets_im_center_star.push_back(p);
    c.targets_im_bottom_star.push_back(p);
    c.targets_d_star.push_back(0);
    geometry_msgs::Quaternion q;
    c.targets_orientation_star.push_back(q);
    c.weights.w_R_targets.push_back(0);
    c.weights.w_img_targets.push_back(w);
    c.weights.w_d_targets.push_back(0);
  }

  int sequence = req.sequence;
  if (sequence == 1)
  {
    for (int i = 0; i < targets_names.size(); i++)
    {

      // targets_relative is relative to drone
      Eigen::Matrix<double, 3, 3> wRtarget = cinempc::quat_to_R_matrix<double>(req.world_rotations_target.at(i));
      cinempc::RPY<double> RPY_target = cinempc::R_matrix_to_RPY<double>(wRtarget);

      // JDoF
      c.dn_star = 0;
      c.weights.w_dn = 0;

      c.df_star = 0;
      c.weights.w_df = 0;

      //Jim
      c.targets_im_top_star.at(i).x = 0;
      c.weights.w_img_targets.at(i).x = 0;
      c.targets_im_top_star.at(i).y = 0;  
      c.weights.w_img_targets.at(i).y_top = 0;
      c.targets_im_bottom_star.at(i).y = 0;  
      c.weights.w_img_targets.at(i).y_bottom = 0.;         
      c.targets_im_center_star.at(i).y = 0; 
      c.weights.w_img_targets.at(i).y_center = 0;            

      //Jp
      c.targets_d_star.at(i) = 0;
      c.weights.w_d_targets.at(i) = 0;

      cinempc::RPY<double> relative_rotation = cinempc::R_matrix_to_RPY<double>(
          cinempc::RPY_to_R_matrix<double>(0, 0, 0).transpose() * wRtarget);

      tf2::Quaternion quaternion_tf2;
      quaternion_tf2.setRPY(relative_rotation.roll, relative_rotation.pitch, relative_rotation.yaw);
      geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
      c.targets_orientation_star.at(0) = quaternion;
      c.weights.w_R_targets.at(0) = 100;

      //Jf
      c.focal_star = 0;
      c.weights.w_focal = 0;
    }
  }
  res.contraints = c;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cinempc_user");

  ros::NodeHandle n;
  ros::ServiceServer service =
      n.advertiseService<cinempc::GetUserConstraints::Request, cinempc::GetUserConstraints::Response>(
          "/cinempc/"
          "user_node/"
          "get_constraints",
          boost::bind(&getConstraints, _1, _2));
  ros::spin();

  return 0;
}