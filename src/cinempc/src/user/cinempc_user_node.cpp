#include <cinempc/CineMPCCommon.h>
#include <cinempc/GetUserConstraints.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include "ros/ros.h"

double desired_far = 0;
bool far_ascending = true;
double desired_focal = 35;
double weight_far = 0;
bool getConstraints(cinempc::GetUserConstraints::Request &req, cinempc::GetUserConstraints::Response &res)
{
  cinempc::Constraints c;

  // targets_relative is relative to drone
  Eigen::Matrix<double, 3, 3> wRtarget_perception = cinempc::quatToRMatrix<double>(req.world_rotations_target.at(0));

  Eigen::Matrix<double, 3, 3> wRtarget_gt = cinempc::RPYtoRMatrix<double>(0, 0, target_yaw_gt);
  Eigen::Matrix<double, 3, 3> wRtarget;
  if (use_perception && !static_target)
  {
    wRtarget = wRtarget_perception;
  }
  else
  {
    wRtarget = wRtarget_gt;
  }
  cinempc::RPY<double> RPY_target = cinempc::RMatrixtoRPY<double>(wRtarget);

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
    // starting target. From front preseting target focused and mid-body
    c.dn_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_top.at(0).position.x,
                                                        req.targets_relative.at(0).poses_top.at(0).position.y, 0, 0)) -
        3;
    c.weights.w_dn = 10;
    c.df_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_top.at(0).position.x,
                                                        req.targets_relative.at(0).poses_top.at(0).position.y, 0, 0)) +
        3;
    c.weights.w_df = 0;

    c.targets_im_top_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 0.25;            // 10;                      // 1;                       // 1 *
    c.targets_im_top_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y_top = 0.25;        // 10;                      // 1;                       // 1 *
    c.targets_im_bottom_star.at(0).y = -1;             // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y_bottom = 0;        // 1;                       // 1 * 1;
    c.targets_im_center_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y_center = 0.25;          // 1;                       // 1 * 1;

    c.targets_d_star.at(0) = -1;
    c.weights.w_d_targets.at(0) = 0;  // 10;  // 1 * 1;
    cinempc::RPY<double> relative = cinempc::RMatrixtoRPY<double>(
        cinempc::RPYtoRMatrix<double>(RPY_target.roll, RPY_target.pitch, RPY_target.yaw - PI).transpose() * wRtarget);

    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 500;

    c.focal_star = desired_focal;
    c.weights.w_focal = 10;
  }
  else if (sequence == 2)
  {
    // starting target. From front preseting target focused and mid-body
    c.dn_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_top.at(0).position.x,
                                                        req.targets_relative.at(0).poses_top.at(0).position.y, 0, 0)) -
        3;
    c.weights.w_dn = 15;
    c.df_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_top.at(0).position.x,
                                                        req.targets_relative.at(0).poses_top.at(0).position.y, 0, 0)) +
        50;

    c.weights.w_df = 0;

    c.targets_im_top_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 0.25;            // 10;                      // 1;                       // 1 * 1;
    c.targets_im_top_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y_top = 0.5;         // 10;                      // 1;                       // 1 * 1;
    c.targets_im_bottom_star.at(0).y = -1;             // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y_bottom = 0;        // 1;                       // 1 * 1;
    c.targets_im_center_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y_center = 0.5;

    c.targets_d_star.at(0) = -1;
    c.weights.w_d_targets.at(0) = 0;  // 10;  // 1 * 1;
    cinempc::RPY<double> relative = cinempc::RMatrixtoRPY<double>(
        cinempc::RPYtoRMatrix<double>(RPY_target.roll, RPY_target.pitch, RPY_target.yaw - PI).transpose() * wRtarget);

    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 500;
    if (desired_focal < 450)
    {
      desired_focal += 5;
    }
    c.focal_star = desired_focal;
    c.weights.w_focal = 0.75;
  }
  else if (sequence == 3)
  {
    // starting target. From front preseting target focused and mid-body
    c.dn_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_top.at(0).position.x,
                                                        req.targets_relative.at(0).poses_top.at(0).position.y, 0, 0)) -
        3;
    c.weights.w_dn = 15;
    if (far_ascending)
    {
      if (desired_far < 55)
      {
        desired_far += 1;
      }
      else if (desired_far >= 55)
      {
        far_ascending = false;
      }
    }
    else
    {
      if (desired_far > 1)
      {
        desired_far -= 2;
      }
    }
    c.df_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_top.at(0).position.x,
                                                        req.targets_relative.at(0).poses_top.at(0).position.y, 0, 0)) +
        desired_far;

    c.weights.w_df = 10;

    c.targets_im_top_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 0.25;            // 10;                      // 1;                       // 1 * 1;
    c.targets_im_top_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y_top = 0.5;         // 10;                      // 1;                       // 1 * 1;
    c.targets_im_bottom_star.at(0).y = -1;             // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y_bottom = 0;        // 1;                       // 1 * 1;
    c.targets_im_center_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y_center = 0.5;

    c.targets_d_star.at(0) = -1;
    c.weights.w_d_targets.at(0) = 0;  // 10;  // 1 * 1;
    cinempc::RPY<double> relative = cinempc::RMatrixtoRPY<double>(
        cinempc::RPYtoRMatrix<double>(RPY_target.roll, RPY_target.pitch, RPY_target.yaw - PI).transpose() * wRtarget);

    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 500;
    if (desired_focal < 500)
    {
      desired_focal += 5;
    }
    c.focal_star = desired_focal;
    c.weights.w_focal = 0.75;
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