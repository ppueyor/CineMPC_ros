
#include <QuaternionConverters.h>
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

  // targets_relative is relative to drone
  Eigen::Matrix<double, 3, 3> wRboy_perception = cinempc::quatToRMatrix<double>(
      cinempc::calculate_world_pose_from_relative<double>(req.targets_relative.at(0).poses_up.at(0), req.drone_pose)
          .orientation);

  Eigen::Matrix<double, 3, 3> dRboy_perception =
      cinempc::quatToRMatrix<double>(req.targets_relative.at(0).poses_up.at(0).orientation);

  Eigen::Matrix<double, 3, 3> wRboy = cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw);

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
        1;
    c.weights.w_dn = 0;
    c.df_star =
        abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets_relative.at(0).poses_up.at(0).position.x,
                                                        req.targets_relative.at(0).poses_up.at(0).position.y, 0, 0)) +
        4;
    c.weights.w_df = 0;

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
    c.weights.w_d_targets.at(0) = 10;  // 1 * 1;
    cinempc::RPY<double> relative = cinempc::RMatrixtoRPY<double>(
        cinempc::RPYtoRMatrix<double>(0, -0.5, RPY_boy_perc.yaw - PI / 2).transpose() * wRboy_perception);

    std::cout << std::endl
              << "RELATIVE STAR: " << std::endl
              << relative.roll << std::endl
              << relative.pitch << std::endl;

    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 5000 * 20;
  }
  // c.targets_d_star.at(1) = -1;
  // c.weights.w_d_targets.at(1) = 0;
  // c.p_girl.R = RPY(0, 0, 0);
  // c.weights.w_R_targets.at(1) = 0;
  // }
  // else if (sequence == 2 || sequence == 2.5)
  // {
  //   // Rotatind around boy. From right (90º) presenting boy focused and full-body. Boy centered
  //   float weight_y = 1;
  //   // if (change_sequence_index <= 20)
  //   // {
  //   //   weight_y = 0.05 * change_sequence_index;
  //   //   change_sequence_index++;
  //   //   cout << "-------------------------------------" << endl
  //   //        << "------    we:    " << weight_y << "       ------" << endl
  //   //        << "-------------------------------------" << endl;
  //   // }
  //   // starting boy. From front preseting boy focused and mid-body
  //   c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets.at(0).poses_up.at(0).position.x,
  //                                                               req.targets.at(0).poses_up.at(0).position.y, 0, 0)) -
  //               10;
  //   c.weights.w_dn = 10 * 2;
  //   c.df_star = 0;
  //   c.weights.w_df = 10 * 1;

  //   c.targets_im_up_star.at(0).x = image_x_center;
  //   c.weights.w_img_targets.at(0).x = 1 * 2;
  //   c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).y = 1 * 2.5;
  //   c.targets_im_down_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).z = weight_y;
  //   c.targets_im_up_star.at(1).x = 0;
  //   c.weights.w_img_targets.at(1).x = 0;
  //   c.targets_im_up_star.at(1).y = 0;
  //   c.weights.w_img_targets.at(1).y = 0;
  //   c.targets_im_down_star.at(1).y = -1;
  //   c.weights.w_img_targets.at(1).z = 0;

  //   c.targets_d_star.at(0) = 5;
  //   c.weights.w_d_targets.at(0) = 100 * 1;
  //   cinempc::RPY<double> relative =
  //       cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, -0.3, subject_yaw - PI / 2).transpose() *
  //       wRboy);
  //   tf2::Quaternion quaternion_tf2;
  //   quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
  //   geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  //   c.targets_orientation_star.at(0) = quaternion;
  //   c.weights.w_R_targets.at(0) = 5000 * 2;

  //   c.targets_d_star.at(1) = -1;
  //   c.weights.w_R_targets.at(0) = 0;
  //   // c.p_girl.R = RPY(0, 0, 0);
  //   c.weights.w_R_targets.at(1) = 0;
  // }
  // else if (sequence == 3)
  // {
  //   // Zoom in boy. From behind (180º) presenting boy focused and mid-body. Boy centered
  //   c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets.at(0).poses_up.at(0).position.x,
  //                                                               req.targets.at(0).poses_up.at(0).position.y, 0, 0)) -
  //               2.5;
  //   c.weights.w_dn = 10 * 5;
  //   c.df_star = abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets.at(0).poses_up.at(0).position.x,
  //                                                               req.targets.at(0).poses_up.at(0).position.y, 0, 0)) +
  //               1;
  //   c.weights.w_df = 10 * 5;

  //   c.targets_im_up_star.at(0).x = image_x_center;
  //   c.weights.w_img_targets.at(0).x = 1 * 1;
  //   c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).y = 1 * 2;
  //   c.targets_im_down_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).z = 1 * 1;
  //   c.targets_im_up_star.at(1).x = 0;
  //   c.weights.w_img_targets.at(1).x = 0;
  //   c.targets_im_up_star.at(1).y = -1;
  //   c.weights.w_img_targets.at(1).y = 0;
  //   c.targets_im_down_star.at(1).y = -1;
  //   c.weights.w_img_targets.at(1).z = 0;

  //   c.targets_d_star.at(0) = 5;
  //   c.weights.w_d_targets.at(0) = 1 * 2;
  //   cinempc::RPY<double> relative =
  //       cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw - PI).transpose() * wRboy);
  //   tf2::Quaternion quaternion_tf2;
  //   quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
  //   geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  //   c.targets_orientation_star.at(0) = quaternion;
  //   c.weights.w_R_targets.at(0) = 5000 * 1;

  //   c.targets_d_star.at(1) = -1;
  //   c.weights.w_R_targets.at(0) = 0;
  //   // c.p_girl.R = RPY(0, 0, 0);
  //   c.weights.w_R_targets.at(1) = 0;
  // }
  // else if (sequence == 4)
  // {
  //   // Presenting girl. From behind (180º) presenting boy and girl focused and full-body of boy. Boy and girl in
  //   thirds c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets.at(0).poses_up.at(0).position.x,
  //                                                               req.targets.at(0).poses_up.at(0).position.y, 0, 0)) -
  //               5;
  //   c.weights.w_dn = 10 * 1;
  //   c.df_star = 0;
  //   c.weights.w_df = 0;

  //   c.targets_im_up_star.at(0).x = image_x_third_left;
  //   c.weights.w_img_targets.at(0).x = 1 * 1;
  //   c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).y = 1 * 2;
  //   c.targets_im_down_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).z = 1 * 1;
  //   c.targets_im_up_star.at(1).x = image_x_third_right;
  //   c.weights.w_img_targets.at(1).x = 1;
  //   c.targets_im_up_star.at(1).y = image_y_third_up;
  //   c.weights.w_img_targets.at(1).y = 1 * 2;
  //   c.targets_im_down_star.at(1).y = -1;
  //   c.weights.w_img_targets.at(1).z = 0;

  //   c.targets_d_star.at(0) = 5;
  //   c.weights.w_d_targets.at(0) = 1 * 3;
  //   cinempc::RPY<double> relative = cinempc::RMatrixtoRPY<double>(
  //       cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw - PI / 2 - 0.15).transpose() * wRboy);
  //   tf2::Quaternion quaternion_tf2;
  //   quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
  //   geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  //   c.targets_orientation_star.at(0) = quaternion;
  //   c.weights.w_R_targets.at(0) = 5000 * 0.5;

  //   c.targets_d_star.at(1) = -1;
  //   c.weights.w_R_targets.at(0) = 0;
  //   // c.p_girl.R = RPY(0, 0, 0);
  //   c.weights.w_R_targets.at(1) = 0;
  // }
  // else if (sequence == 5)
  // {
  //   // Highligh girl. From behind (180º) presenting girl focused and boy not focused and full-body of girl. Boy and
  //   girl
  //   // in thirds
  //   c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets.at(1).poses_up.at(0).position.x,
  //                                                               req.targets.at(1).poses_up.at(0).position.y, 0, 0)) -
  //               1;
  //   c.weights.w_dn = 10 * 1;
  //   c.df_star = abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets.at(1).poses_up.at(0).position.x,
  //                                                               req.targets.at(0).poses_up.at(0).position.y, 0, 0)) +
  //               20;
  //   c.weights.w_df = 5;

  //   c.targets_im_up_star.at(0).x = image_x_third_left;
  //   c.weights.w_img_targets.at(0).x = 1 * 1;
  //   c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).y = 1 * 1;
  //   c.targets_im_down_star.at(0).y = 0;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).z = 0;

  //   c.targets_im_up_star.at(1).x = image_x_third_right;
  //   c.weights.w_img_targets.at(1).x = 1 * 2;
  //   c.targets_im_up_star.at(1).y = image_y_third_up;
  //   c.weights.w_img_targets.at(1).y = 1 * 2;
  //   c.targets_im_down_star.at(1).y = image_y_third_down;
  //   c.weights.w_img_targets.at(1).z = 1;

  //   c.targets_d_star.at(0) = 5;
  //   c.weights.w_d_targets.at(0) = 1 * 2;
  //   cinempc::RPY<double> relative = cinempc::RMatrixtoRPY<double>(
  //       cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw - PI / 2 - 0.17).transpose() * wRboy);
  //   tf2::Quaternion quaternion_tf2;
  //   quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
  //   geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  //   c.targets_orientation_star.at(0) = quaternion;
  //   c.weights.w_R_targets.at(0) = 5000 * 0.5;

  //   c.targets_d_star.at(1) = -1;
  //   c.weights.w_R_targets.at(0) = 0;
  //   // c.p_girl.R = RPY(0, 0, 0);
  //   c.weights.w_R_targets.at(1) = 0;
  // }
  // else if (sequence == 6)
  // {
  //   // Focus just in girl. From front of girl (180º) focus on girl focused  focused and mid-body of girl centered.
  //   Girl c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets.at(1).poses_up.at(0).position.x,
  //                                                               req.targets.at(1).poses_up.at(0).position.y, 0, 0)) -
  //               1;
  //   c.weights.w_dn = 10 * 1;
  //   c.df_star = abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets.at(1).poses_up.at(0).position.x,
  //                                                               req.targets.at(1).poses_up.at(0).position.y, 0, 0)) +
  //               35;
  //   c.weights.w_df = 10;

  //   c.targets_im_up_star.at(0).x = image_x_third_left;
  //   c.weights.w_img_targets.at(0).x = 0;
  //   c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).y = 0;
  //   c.targets_im_down_star.at(0).y = 0;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).z = 0;

  //   c.targets_im_up_star.at(1).x = image_x_center;
  //   c.weights.w_img_targets.at(1).x = 1 * 2;
  //   c.targets_im_up_star.at(1).y = image_y_third_up;
  //   c.weights.w_img_targets.at(1).y = 1 * 2;
  //   c.targets_im_down_star.at(1).y = image_y_third_down;
  //   c.weights.w_img_targets.at(1).z = 1;

  //   c.targets_d_star.at(0) = 5;
  //   c.weights.w_d_targets.at(0) = 0;
  //   // c.targets_orientation_star.at(0) = quaternion;
  //   c.weights.w_R_targets.at(0) = 0;

  //   c.targets_d_star.at(1) = -1;
  //   c.weights.w_R_targets.at(0) = 0;
  //   cinempc::RPY<double> relative_girl =
  //       cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, -0.1, PI - PI - PI / 4).transpose() * wRw);
  //   tf2::Quaternion quaternion_tf2_girl;
  //   quaternion_tf2_girl.setRPY(relative_girl.roll, relative_girl.pitch, relative_girl.yaw);
  //   geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2_girl);
  //   c.targets_orientation_star.at(1) = quaternion;
  //   c.weights.w_R_targets.at(1) = 10000;
  // }
  // else if (sequence == 7)
  // {
  //   // Focus just in girl. From front of girl (180º) focus on girl focused  focused and mid-body of girl centered.
  //   Girl
  //   // centered Highligh girl. From behind (180º) presenting girl focused and boy not focused and full-body of girl.
  //   Boy

  //   c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(req.targets.at(1).poses_up.at(0).position.x,
  //                                                               req.targets.at(1).poses_up.at(0).position.y, 0, 0)) -
  //               5;
  //   c.weights.w_dn = 1;
  //   c.df_star = 0;
  //   c.weights.w_df = 0;

  //   c.targets_im_up_star.at(0).x = 0;
  //   c.weights.w_img_targets.at(0).x = 0;
  //   c.targets_im_up_star.at(0).y = 0;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).y = 0;
  //   c.targets_im_down_star.at(0).y = 0;  // mid-body (control with calculations of positions)
  //   c.weights.w_img_targets.at(0).z = 0;

  //   c.targets_im_up_star.at(1).x = image_x_center;
  //   c.weights.w_img_targets.at(1).x = 1 * 2;
  //   c.targets_im_up_star.at(1).y = image_y_third_up;
  //   c.weights.w_img_targets.at(1).y = 1;
  //   c.targets_im_down_star.at(1).y = image_y_third_down;
  //   c.weights.w_img_targets.at(1).z = 1;

  //   c.targets_d_star.at(0) = 5;
  //   c.weights.w_d_targets.at(0) = 0;
  //   // c.targets_orientation_star.at(0) = quaternion;
  //   c.weights.w_R_targets.at(0) = 0;

  //   c.targets_d_star.at(1) = 5;
  //   c.weights.w_R_targets.at(0) = 1;

  //   cinempc::RPY<double> relative_girl =
  //       cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, -0.3, PI - PI - 3 * PI / 4).transpose() *
  //       wRw);
  //   tf2::Quaternion quaternion_tf2_girl;
  //   quaternion_tf2_girl.setRPY(relative_girl.roll, relative_girl.pitch, relative_girl.yaw);
  //   geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2_girl);
  //   c.targets_orientation_star.at(1) = quaternion;
  //   c.weights.w_R_targets.at(1) = 10000;
  // }
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
