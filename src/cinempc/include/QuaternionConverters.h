#include <Constants.h>
#include <cinempc/PlotValues.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

namespace cinempc
{
template <typename T>
struct Pixel
{
  T x;
  T y;
  Pixel(T xc, T yc) : x(xc), y(yc)
  {
  }
};

template <typename T>
struct RPY
{
  T roll = 0;
  T pitch = 0;
  T yaw = 0;

  RPY(){};
  RPY(T rollc, T pitchc, T yawc) : roll(rollc), pitch(pitchc), yaw(yawc)
  {
  }
};

template <typename T>
Eigen::Matrix<T, 3, 3> RPYtoRMatrix(T roll, T pitch, T yaw)
{
  // Eigen::Quaterniond quat = VectorMath::toQuaternion(0,0,CppAD::Value(yaw)).Quaternion;
  Eigen::AngleAxis<T> rollAngle(roll, Eigen::Matrix<T, 1, 3>::UnitX());
  Eigen::AngleAxis<T> pitchAngle(pitch, Eigen::Matrix<T, 1, 3>::UnitY());
  Eigen::AngleAxis<T> yawAngle(yaw, Eigen::Matrix<T, 1, 3>::UnitZ());

  Eigen::Matrix<T, 3, 3> R;

  Eigen::Quaternion<T> q = yawAngle * rollAngle * pitchAngle;

  R = q.matrix();

  return (R);
}

template <typename T>
RPY<T> RMatrixtoRPY(Eigen::Matrix<T, 3, 3> R)
{
  T roll = asin(R(2, 1));
  T pitch = -atan2(R(2, 0), R(2, 2));
  T yaw = -atan2(R(0, 1), R(1, 1));

  RPY<T> RPY;
  RPY.roll = roll;
  RPY.pitch = pitch;
  RPY.yaw = yaw;

  return RPY;
}

template <typename T>
geometry_msgs::Quaternion RPYToQuat(T roll, T pitch, T yaw)
{
  tf2::Quaternion quaternion_tf2;
  quaternion_tf2.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  return quaternion;
}

template <typename T>
Eigen::Matrix<T, 3, 3> quatToRMatrix(geometry_msgs::Quaternion q)
{
  double roll, pitch, yaw;
  tf2::Quaternion quat_tf;
  tf2::fromMsg(q, quat_tf);
  Eigen::Matrix<T, 3, 3> mat_res;
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  return RPYtoRMatrix<T>(roll, pitch, yaw);
}

template <typename T>
geometry_msgs::Quaternion RMatrixToQuat(Eigen::Matrix<T, 3, 3> matrix)
{
  tf2::Quaternion quaternion_tf;
  RPY<T> rpy = RMatrixtoRPY(matrix);
  quaternion_tf.setRPY(rpy.roll, rpy.pitch, rpy.yaw);
  quaternion_tf.normalize();

  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quaternion_tf);

  return quat_msg;
}

template <typename T>
RPY<T> quatToRPY(geometry_msgs::Quaternion quat_msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion quat_tf;
  tf2::fromMsg(quat_msg, quat_tf);
  Eigen::Matrix<T, 3, 3> mat_res;
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
  RPY<T> rpy;
  rpy.roll = roll;
  rpy.pitch = pitch;
  rpy.yaw = yaw;
  return rpy;
}

template <typename T>
geometry_msgs::Pose calculate_relative_poses_drone_targets(geometry_msgs::Pose drone_pose_target1,
                                                           geometry_msgs::Pose target1_pose_target2)
{
  geometry_msgs::Pose drone_pose_target2;

  Eigen::Matrix<T, 3, 3> drone_R_target2 =
      quatToRMatrix<T>(drone_pose_target1.orientation) * quatToRMatrix<T>(target1_pose_target2.orientation);

  Eigen::Matrix<T, 3, 1> drone_t_target1(drone_pose_target1.position.x, drone_pose_target1.position.y,
                                         drone_pose_target1.position.z);
  Eigen::Matrix<T, 3, 1> target1_t_target2(target1_pose_target2.position.x, target1_pose_target2.position.y,
                                           target1_pose_target2.position.z);

  Eigen::Matrix<T, 3, 1> drone_t_target2 =
      drone_t_target1 + quatToRMatrix<T>(drone_pose_target1.orientation) * target1_t_target2;

  drone_pose_target2.orientation = RMatrixToQuat(drone_R_target2);

  drone_pose_target2.position.x = drone_t_target2(0);
  drone_pose_target2.position.y = drone_t_target2(1);
  drone_pose_target2.position.z = drone_t_target2(2);

  return drone_pose_target2;
}

template <typename T>
geometry_msgs::Pose calculate_relative_pose_drone_person(geometry_msgs::Pose target_pose,
                                                         geometry_msgs::Pose drone_pose)
{
  geometry_msgs::Pose relative_pose;

  Eigen::Matrix<T, 3, 3> wRd = quatToRMatrix<T>(drone_pose.orientation);
  Eigen::Matrix<T, 3, 3> wRp = quatToRMatrix<T>(target_pose.orientation);
  Eigen::Matrix<T, 3, 3> dRp = wRd.transpose() * wRp;

  Eigen::Matrix<T, 3, 1> wtp(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  Eigen::Matrix<T, 3, 1> wtd(drone_pose.position.x, drone_pose.position.y, drone_pose.position.z);

  Eigen::Matrix<T, 3, 1> dtp = wRd.transpose() * (wtp - wtd);

  relative_pose.position.x = dtp(0);
  relative_pose.position.y = dtp(1);
  relative_pose.position.z = dtp(2);

  relative_pose.orientation = RMatrixToQuat(dRp);

  return (relative_pose);
}

template <typename T>
geometry_msgs::Pose calculate_world_pose_from_relative(geometry_msgs::Pose drone_pose, geometry_msgs::Pose target_pose)
{
  geometry_msgs::Pose drone_distance_relative;

  Eigen::Matrix<T, 3, 3> wRd = quatToRMatrix<T>(drone_pose.orientation);
  Eigen::Matrix<T, 3, 3> wRs = wRd * quatToRMatrix<T>(target_pose.orientation);
  Eigen::Matrix<T, 3, 1> wts;

  Eigen::Matrix<T, 3, 1> dts(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  Eigen::Matrix<T, 3, 1> wtd(drone_pose.position.x, drone_pose.position.y, drone_pose.position.z);

  wts = wtd + wRd * dts;

  drone_distance_relative.position.x = wts(0);
  drone_distance_relative.position.y = wts(1);
  drone_distance_relative.position.z = wts(2);

  drone_distance_relative.orientation = RMatrixToQuat(wRs);

  return (drone_distance_relative);
}

template <typename T>
geometry_msgs::Pose calculate_world_pose_from_relative(geometry_msgs::Pose drone_pose, geometry_msgs::Pose target_pose,
                                                       bool from_zero)
{
  geometry_msgs::Pose drone_distance_relative;

  Eigen::Matrix<T, 3, 3> wRd = quatToRMatrix<T>(drone_pose.orientation);
  Eigen::Matrix<T, 3, 3> wRs = wRd * quatToRMatrix<T>(target_pose.orientation);
  Eigen::Matrix<T, 3, 1> wts;

  Eigen::Matrix<T, 3, 1> dts(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  Eigen::Matrix<T, 3, 1> wtd(drone_pose.position.x, drone_pose.position.y, drone_pose.position.z);

  if (from_zero)
  {
    wts = wRd * dts;
  }
  else
  {
    wts = wtd + wRd * dts;
  }
  drone_distance_relative.position.x = wts(0);
  drone_distance_relative.position.y = wts(1);
  drone_distance_relative.position.z = wts(2);

  drone_distance_relative.orientation = RMatrixToQuat(wRs);

  return (drone_distance_relative);
}

// For now we assume that the target will be rotated as the world
template <typename T>
geometry_msgs::Pose drone_relative_position_from_image(T focal_length_mm, T u_px, T v_px, T depth_mm,
                                                       geometry_msgs::Quaternion world_drone_orientation,
                                                       geometry_msgs::Quaternion world_target_orientation)
{
  T x0_mm = sensor_width_mm / 2;
  T y0_mm = sensor_height_mm / 2;

  T u_mm = u_px * sensor_width_mm / picture_width_px;
  T v_mm = v_px * sensor_height_mm / picture_height_px;
  T dx_mm = (depth_mm) * (u_mm - x0_mm) / focal_length_mm;

  T dy_mm = (depth_mm) * (v_mm - y0_mm) / focal_length_mm;

  T dx_m = dx_mm / 1000;
  T dy_m = dy_mm / 1000;

  geometry_msgs::Point rPs_point;
  rPs_point.x = dx_m;
  rPs_point.y = dy_m;
  rPs_point.z = depth_mm / 1000;

  Eigen::Matrix<T, 3, 1> dPt(rPs_point.z, rPs_point.x, rPs_point.y);

  geometry_msgs::Quaternion dRt = RMatrixToQuat<T>(quatToRMatrix<T>(world_drone_orientation).transpose() *
                                                   quatToRMatrix<T>(world_target_orientation));

  geometry_msgs::Pose dTp;
  dTp.position.x = dPt(0);
  dTp.position.y = dPt(1);
  dTp.position.z = dPt(2);

  dTp.orientation = dRt;

  return dTp;
}

// For now we assume that the target will be rotated as the world
template <typename T>
geometry_msgs::Pose convertDroneToWorld(T focal_length_mm, T u_px, T v_px, T depth_mm,
                                        geometry_msgs::Quaternion world_drone_orientation,
                                        geometry_msgs::Quaternion world_target_orientation)
{
  T x0_mm = sensor_width_mm / 2;
  T y0_mm = sensor_height_mm / 2;

  T u_mm = u_px * sensor_width_mm / picture_width_px;
  T v_mm = v_px * sensor_height_mm / picture_height_px;
  T dx_mm = (depth_mm) * (u_mm - x0_mm) / focal_length_mm;

  T dy_mm = (depth_mm) * (v_mm - y0_mm) / focal_length_mm;

  T dx_m = dx_mm / 1000;
  T dy_m = dy_mm / 1000;

  geometry_msgs::Point rPs_point;
  rPs_point.x = dx_m;
  rPs_point.y = dy_m;
  rPs_point.z = depth_mm / 1000;

  Eigen::Matrix<T, 3, 1> dPt(rPs_point.z, rPs_point.x, rPs_point.y);

  geometry_msgs::Quaternion dRt = RMatrixToQuat<T>(quatToRMatrix<T>(world_drone_orientation).transpose() *
                                                   quatToRMatrix<T>(world_target_orientation));

  geometry_msgs::Pose dTp;
  dTp.position.x = dPt(0);
  dTp.position.y = dPt(1);
  dTp.position.z = dPt(2);

  dTp.orientation = dRt;

  return dTp;
}

// Function to calculate distance between two points (focus distance)

template <typename T>
T calculateDistanceTo2DPoint(T x1, T y1, T x2, T y2)
{
  // Calculating distance
  return (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0));
}
// Function to calculate distance between two points (focus distance)

template <typename T>
T calculateDistanceTo3DPoint(T x1, T y1, T z1, T x2, T y2, T z2)
{
  // Calculating distance
  return (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2)));
}

std::string plotValues(cinempc::PlotValues plot_values, bool header)
{
  std::ostringstream result;
  if (header)
  {
    result << "Time"
           << ","
           << "Sequence"
           << ","
           << "world_x_gt"
           << ","
           << "world_y_gt"
           << ","
           << "world_z_gt"
           << ","
           << "world_x_perception"
           << ","
           << "world_y_perception"
           << ","
           << "world_z_perception"
           << ","
           << "world_x_kf"
           << ","
           << "world_y_kf"
           << ","
           << "world_z_kf"
           << ","
           << "d_gt"
           << ","
           << "v_x_kf"
           << ","
           << "v_y_kf"
           << ","
           << "v_z_kf"
           << ","
           << "pitch_gt"
           << ","
           << "yaw_gt"
           << ","
           << "pitch_perception"
           << ","
           << "yaw_perception"
           << ","
           << "focal_length"
           << ","
           << "focus_distance"
           << ","
           << "aperture"
           << ","
           << "dn"
           << ","
           << "df"
           << ","
           << "im_u"
           << ","
           << "im_v_up"
           << ","
           << "im_v_center"
           << ","
           << "im_v_down"
           << ","
           << "cost"
           << ","
           << "Jp"
           << ","
           << "Jim"
           << ","
           << "JDoF"
           << ","
           << "Jf"
           << ","
           << "focal_length_d"
           << ","
           << "dn_d"
           << ","
           << "df_d"
           << ","
           << "relative_roll"
           << ","
           << "relative_pitch"
           << ","
           << "relative_yaw"
           << ","
           << "relative_roll_d"
           << ","
           << "relative_pitch_d"
           << ","
           << "relative_yaw_d"
           << ","
           << "d_d"
           << ","
           << "im_u_d"
           << ","
           << "im_v_up_d"
           << ","
           << "im_v_center_d"
           << ","
           << "im_v_down_d"
           << ","
           << "drone_x"
           << ","
           << "drone_y"
           << ","
           << "drone_z" << std::endl;
  }
  else
  {
    result << plot_values.time_ms << "," << plot_values.sequence << "," << plot_values.target_world_gt.x << ","
           << plot_values.target_world_gt.y << "," << plot_values.target_world_gt.z << ","
           << plot_values.target_world_perception.x << "," << plot_values.target_world_perception.y << ","
           << plot_values.target_world_perception.z << "," << plot_values.target_world_kf.x << ","
           << plot_values.target_world_kf.y << "," << plot_values.target_world_kf.z << "," << plot_values.d_gt << ","
           << plot_values.v_kf.x << "," << plot_values.v_kf.y << "," << plot_values.v_kf.z << ","
           << cinempc::quatToRPY<double>(plot_values.target_rot_gt).pitch << ","
           << cinempc::quatToRPY<double>(plot_values.target_rot_gt).yaw << ","
           << cinempc::quatToRPY<double>(plot_values.target_rot_perception).pitch << ","
           << cinempc::quatToRPY<double>(plot_values.target_rot_perception).yaw << ","
           << plot_values.mpc_plot_values.intrinsics_camera.focal_length << ","
           << plot_values.mpc_plot_values.intrinsics_camera.focus_distance << ","
           << plot_values.mpc_plot_values.intrinsics_camera.aperture << "," << plot_values.mpc_plot_values.dn << ","
           << plot_values.mpc_plot_values.df << "," << plot_values.mpc_plot_values.im_u << ","
           << plot_values.mpc_plot_values.im_v_up << "," << plot_values.mpc_plot_values.im_v_center << ","
           << plot_values.mpc_plot_values.im_v_down << "," << plot_values.mpc_plot_values.cost << ","
           << plot_values.mpc_plot_values.Jp << "," << plot_values.mpc_plot_values.Jim << ","
           << plot_values.mpc_plot_values.JDoF << "," << plot_values.mpc_plot_values.JFoc << ","
           << plot_values.constraints.focal_star << "," << plot_values.constraints.dn_star << ","
           << plot_values.constraints.df_star << "," << plot_values.mpc_plot_values.relative_roll << ","
           << plot_values.mpc_plot_values.relative_pitch << "," << plot_values.mpc_plot_values.relative_yaw << ","
           << cinempc::quatToRPY<double>(plot_values.constraints.targets_orientation_star.at(0)).roll << ","
           << cinempc::quatToRPY<double>(plot_values.constraints.targets_orientation_star.at(0)).pitch << ","
           << cinempc::quatToRPY<double>(plot_values.constraints.targets_orientation_star.at(0)).yaw << ","
           << plot_values.constraints.targets_d_star.at(0) << "," << plot_values.constraints.targets_im_top_star.at(0).x
           << "," << plot_values.constraints.targets_im_top_star.at(0).y << ","
           << plot_values.constraints.targets_im_center_star.at(0).y << ","
           << plot_values.constraints.targets_im_bottom_star.at(0).y << "," << plot_values.drone_position_gt.x << ","
           << plot_values.drone_position_gt.y << "," << plot_values.drone_position_gt.z << std::endl;
  }
  return result.str();
}
};  // namespace cinempc
