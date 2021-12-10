
#include <QuaternionConverters.h>
#include <cinempc/GetUserConstraints.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include "ros/ros.h"

double desired_pitch = -0.3;
bool getConstraints(cinempc::GetUserConstraints::Request &req, cinempc::GetUserConstraints::Response &res)
{
  cinempc::Constraints c;

  // targets_relative is relative to drone
  Eigen::Matrix<double, 3, 3> wRboy_perception = cinempc::quatToRMatrix<double>(
      cinempc::calculate_world_pose_from_relative<double>(req.targets_relative.at(0).poses_up.at(0), req.drone_pose)
          .orientation);

  Eigen::Matrix<double, 3, 3> dRboy_perception =
      cinempc::quatToRMatrix<double>(req.targets_relative.at(0).poses_up.at(0).orientation);

  Eigen::Matrix<double, 3, 3> wRboy = cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw_gt);

  cinempc::RPY<double> RPY_boy = cinempc::RMatrixtoRPY<double>(wRboy);
  cinempc::RPY<double> RPY_boy__rel_perc = cinempc::RMatrixtoRPY<double>(dRboy_perception);
  cinempc::RPY<double> RPY_boy_perc = cinempc::RMatrixtoRPY<double>(wRboy_perception);

  std::cout << std::endl
            << "  RPY BOY   " << std::endl
            << std::endl
            << RPY_boy.roll << std::endl
            << RPY_boy.pitch << std::endl
            << RPY_boy.yaw << "  " << std::endl;

  std::cout << std::endl
            << "  RPY PERCEPTION   " << std::endl
            << std::endl
            << RPY_boy_perc.roll << std::endl
            << RPY_boy_perc.pitch << std::endl
            << RPY_boy_perc.yaw << "  " << std::endl;

  std::cout << std::endl
            << "  RPY RELATIVE PERCEPTION   " << std::endl
            << std::endl
            << RPY_boy__rel_perc.roll << std::endl
            << RPY_boy__rel_perc.pitch << std::endl
            << RPY_boy__rel_perc.yaw << "  " << std::endl;

  for (int i = 0; i < req.targets_relative.size(); i++)
  {
    geometry_msgs::Point p;
    c.targets_im_up_star.push_back(p);
    c.targets_im_down_star.push_back(p);
    c.targets_d_star.push_back(0);
    geometry_msgs::Quaternion q;
    c.targets_orientation_star.push_back(q);
    c.weights.w_R_targets.push_back(0);
    c.weights.w_img_targets.push_back(p);
    c.weights.w_d_targets.push_back(0);
  }

  int sequence = req.sequence;
  if (sequence == 1)
  {
    // starting boy. From front preseting boy focused and mid-body
    c.dn_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_up.at(0).position.x,
                                                        req.targets_relative.at(0).poses_up.at(0).position.y, 0, 0)) -
        5;
    c.weights.w_dn = 10;
    c.df_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_up.at(0).position.x,
                                                        req.targets_relative.at(0).poses_up.at(0).position.y, 0, 0));
    c.weights.w_df = 10;

    c.targets_im_up_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 10;
    ;                                                 // 10;                      // 1;                       // 1 * 1;
    c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 10;             // 10;                      // 1;                       // 1 * 1;
    c.targets_im_down_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 10;                 // 1;                       // 1 * 1;

    // c.targets_im_up_star.at(1).x = -1;
    // c.weights.w_img_targets.at(1).x = 0;
    // c.targets_im_up_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    // c.weights.w_img_targets.at(1).y = 0;
    // c.targets_im_down_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    // c.weights.w_img_targets.at(1).z = 0;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 20;  // 10;  // 1 * 1;
    cinempc::RPY<double> relative =
        cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, 0, RPY_boy.yaw - PI).transpose() * wRboy);

    std::cout << std::endl
              << "RELATIVE STAR: " << std::endl
              << relative.roll << std::endl
              << relative.pitch << std::endl;

    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 5000;

    c.focal_star = 35;
    c.weights.w_focal = 0;
  }

  if (sequence == 2)
  {
    // starting boy. From front preseting boy focused and mid-body
    c.dn_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_up.at(0).position.x,
                                                        req.targets_relative.at(0).poses_up.at(0).position.y, 0, 0)) -
        1;
    c.weights.w_dn = 0;
    c.df_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_up.at(0).position.x,
                                                        req.targets_relative.at(0).poses_up.at(0).position.y, 0, 0)) +
        4;
    c.weights.w_df = 0;

    c.targets_im_up_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 10;
    ;  // 10;                      // 1;                       // 1 * 1;
    c.targets_im_up_star.at(0).y = image_y_third_up + 60;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 10;  // 10;                      // 1;                       // 1 * 1;
    c.targets_im_down_star.at(0).y = image_y_third_down - 60;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 10;                      // 1;                       // 1 * 1;

    // c.targets_im_up_star.at(1).x = -1;
    // c.weights.w_img_targets.at(1).x = 0;
    // c.targets_im_up_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    // c.weights.w_img_targets.at(1).y = 0;
    // c.targets_im_down_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    // c.weights.w_img_targets.at(1).z = 0;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 0;  // 1 * 1;
    cinempc::RPY<double> relative =
        cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, -0.3, RPY_boy.yaw - PI).transpose() * wRboy);

    std::cout << std::endl
              << "RELATIVE STAR: " << std::endl
              << relative.roll << std::endl
              << relative.pitch << std::endl;

    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 50000;

    c.focal_star = 400;
    c.weights.w_focal = 0;
  }

  if (sequence == 3)
  {
    if (desired_pitch < 0)
    {
      desired_pitch += 0.002;
    }
    // starting boy. From front preseting boy focused and mid-body
    c.dn_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_up.at(0).position.x,
                                                        req.targets_relative.at(0).poses_up.at(0).position.y, 0, 0)) -
        1;
    c.weights.w_dn = 0;
    c.df_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_up.at(0).position.x,
                                                        req.targets_relative.at(0).poses_up.at(0).position.y, 0, 0)) +
        999999999999;
    c.weights.w_df = 100;

    c.targets_im_up_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 0;
    ;  // 10;                      // 1;                       // 1 * 1;
    c.targets_im_up_star.at(0).y = image_y_third_up + 60;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 0;  // 10;                      // 1;                       // 1 * 1;
    c.targets_im_down_star.at(0).y = image_y_third_down - 60;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 0;                       // 1;                       // 1 * 1;

    // c.targets_im_up_star.at(1).x = -1;
    // c.weights.w_img_targets.at(1).x = 0;
    // c.targets_im_up_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    // c.weights.w_img_targets.at(1).y = 0;
    // c.targets_im_down_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    // c.weights.w_img_targets.at(1).z = 0;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 0;  // 1 * 1;
    cinempc::RPY<double> relative = cinempc::RMatrixtoRPY<double>(
        cinempc::RPYtoRMatrix<double>(0, desired_pitch, RPY_boy.yaw - PI).transpose() * wRboy);

    std::cout << std::endl
              << "RELATIVE STAR: " << std::endl
              << relative.roll << std::endl
              << relative.pitch << std::endl;

    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 50000;

    c.focal_star = 400;
    c.weights.w_focal = 0;
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
