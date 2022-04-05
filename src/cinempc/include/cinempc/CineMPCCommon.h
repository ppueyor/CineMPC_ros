#include <cinempc/PlotValues.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <user/Constants.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include "std_msgs/Bool.h"

namespace cinempc
{
template <typename T>
struct Pixel
{
  T x;
  T y;
  Pixel(T xc, T yc) : x(xc), y(yc){};
  Pixel()
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
Eigen::Matrix<T, 3, 3> RPY_to_R_matrix(T roll, T pitch, T yaw)
{
  Eigen::AngleAxis<T> rollAngle(roll, Eigen::Matrix<T, 1, 3>::UnitX());
  Eigen::AngleAxis<T> pitchAngle(pitch, Eigen::Matrix<T, 1, 3>::UnitY());
  Eigen::AngleAxis<T> yawAngle(yaw, Eigen::Matrix<T, 1, 3>::UnitZ());

  Eigen::Matrix<T, 3, 3> R;

  Eigen::Quaternion<T> q = yawAngle * rollAngle * pitchAngle;

  R = q.matrix();

  return (R);
}

template <typename T>
RPY<T> R_matrix_to_RPY(Eigen::Matrix<T, 3, 3> R)
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
geometry_msgs::Quaternion RPY_to_quat(T roll, T pitch, T yaw)
{
  tf2::Quaternion quaternion_tf2;
  quaternion_tf2.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  return quaternion;
}

template <typename T>
Eigen::Matrix<T, 3, 3> quat_to_R_matrix(geometry_msgs::Quaternion q)
{
  double roll, pitch, yaw;
  tf2::Quaternion quat_tf;
  tf2::fromMsg(q, quat_tf);
  Eigen::Matrix<T, 3, 3> mat_res;
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  return RPY_to_R_matrix<T>(roll, pitch, yaw);
}

template <typename T>
geometry_msgs::Quaternion R_matrix_to_quat(Eigen::Matrix<T, 3, 3> matrix)
{
  tf2::Quaternion quaternion_tf;
  RPY<T> rpy = R_matrix_to_RPY(matrix);
  quaternion_tf.setRPY(rpy.roll, rpy.pitch, rpy.yaw);
  quaternion_tf.normalize();

  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quaternion_tf);

  return quat_msg;
}

template <typename T>
RPY<T> quat_to_RPY(geometry_msgs::Quaternion quat_msg)
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
geometry_msgs::Pose calculate_relative_pose_drone_target(geometry_msgs::Pose target_pose,
                                                         geometry_msgs::Pose drone_pose)
{
  geometry_msgs::Pose relative_pose;

  Eigen::Matrix<T, 3, 3> wRd = quat_to_R_matrix<T>(drone_pose.orientation);
  Eigen::Matrix<T, 3, 3> wRp = quat_to_R_matrix<T>(target_pose.orientation);
  Eigen::Matrix<T, 3, 3> dRp = wRd.transpose() * wRp;

  Eigen::Matrix<T, 3, 1> wtp(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  Eigen::Matrix<T, 3, 1> wtd(drone_pose.position.x, drone_pose.position.y, drone_pose.position.z);

  Eigen::Matrix<T, 3, 1> dtp = wRd.transpose() * (wtp - wtd);

  relative_pose.position.x = dtp(0);
  relative_pose.position.y = dtp(1);
  relative_pose.position.z = dtp(2);

  relative_pose.orientation = R_matrix_to_quat(dRp);

  return (relative_pose);
}

template <typename T>
geometry_msgs::Pose calculate_drone_world_pose_from_relative(geometry_msgs::Pose drone_pose,
                                                             geometry_msgs::Pose relative_pose)
{
  geometry_msgs::Pose drone_distance_relative;

  Eigen::Matrix<T, 3, 3> wRd = quat_to_R_matrix<T>(drone_pose.orientation);
  Eigen::Matrix<T, 3, 3> wRs = wRd * quat_to_R_matrix<T>(relative_pose.orientation);
  Eigen::Matrix<T, 3, 1> wts;

  Eigen::Matrix<T, 3, 1> dts(relative_pose.position.x, relative_pose.position.y, relative_pose.position.z);
  Eigen::Matrix<T, 3, 1> wtd(drone_pose.position.x, drone_pose.position.y, drone_pose.position.z);

  wts = wtd + wRd * dts;

  drone_distance_relative.position.x = wts(0);
  drone_distance_relative.position.y = wts(1);
  drone_distance_relative.position.z = wts(2);

  drone_distance_relative.orientation = R_matrix_to_quat(wRs);

  return (drone_distance_relative);
}

template <typename T>
geometry_msgs::Pose calculate_drone_world_pose_from_relative(geometry_msgs::Pose drone_pose,
                                                             geometry_msgs::Pose relative_pose, bool from_zero)
{
  geometry_msgs::Pose drone_distance_relative;

  Eigen::Matrix<T, 3, 3> wRd = quat_to_R_matrix<T>(drone_pose.orientation);
  Eigen::Matrix<T, 3, 3> wRs = wRd * quat_to_R_matrix<T>(relative_pose.orientation);
  Eigen::Matrix<T, 3, 1> wts;

  Eigen::Matrix<T, 3, 1> dts(relative_pose.position.x, relative_pose.position.y, relative_pose.position.z);
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

  drone_distance_relative.orientation = R_matrix_to_quat(wRs);

  return (drone_distance_relative);
}

template <typename T>
geometry_msgs::Pose drone_target_relative_position_from_image(T focal_length_mm, T u_px, T v_px, T depth_mm,
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

  geometry_msgs::Quaternion dRt = R_matrix_to_quat<T>(quat_to_R_matrix<T>(world_drone_orientation).transpose() *
                                                      quat_to_R_matrix<T>(world_target_orientation));

  geometry_msgs::Pose dTp;
  dTp.position.x = dPt(0);
  dTp.position.y = dPt(1);
  dTp.position.z = dPt(2);

  dTp.orientation = dRt;

  return dTp;
}

// Function to calculate distance between two 2D points
template <typename T>
T calculateDistanceTo2DPoint(T x1, T y1, T x2, T y2)
{
  // Calculating distance
  return (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0));
}
// Function to calculate distance between two 3D points
template <typename T>
T calculateDistanceTo3DPoint(T x1, T y1, T z1, T x2, T y2, T z2)
{
  // Calculating distance
  return (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2)));
}

template <typename T>
geometry_msgs::Quaternion predict_target_world_orientation_from_velocity(T vx, T vy, T vz)
{
  Eigen::Matrix<T, 3, 1> wvt(vx, vy, vz);
  wvt.normalize();
  Eigen::Matrix<T, 3, 1> g(0, 0, 9.8);
  g.normalize();
  Eigen::Matrix<T, 3, 1> a = g.cross(wvt);
  a.normalize();
  Eigen::Matrix<T, 3, 1> b = wvt.cross(a);
  b.normalize();

  // Now we have R in the world, we suppose all the parts of the target will have the same orientation
  Eigen::Matrix<T, 3, 3> R;
  R.col(0) = wvt;
  R.col(1) = a;
  R.col(2) = b;

  return R_matrix_to_quat<double>(R);
}

template <typename T>
Eigen::Matrix<T, 3, 3> wedge_perator(Eigen::Matrix<T, 3, 1> M)
{
  Eigen::Matrix<T, 3, 3> Wedged;
  Wedged(0, 0) = 0;
  Wedged(0, 1) = -M(2);
  Wedged(0, 2) = M(1);
  Wedged(1, 0) = M(2);
  Wedged(1, 1) = 0;
  Wedged(1, 2) = -M(0);
  Wedged(2, 0) = -M(1);
  Wedged(2, 1) = M(0);
  Wedged(2, 2) = 0;

  return (Wedged);
}

template <typename T>
Eigen::Matrix<T, 3, 3> exp_wedge_operator(Eigen::Matrix<T, 3, 1> M)
{
  Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity(3, 3);

  Eigen::Matrix<T, 3, 1> RotationAxis;
  T rotationAngle;
  if (M(0) == 0 && M(1) == 0 && M(2) == 0)
  {
    rotationAngle = 0;
    RotationAxis = Eigen::Matrix<T, 3, 1>::Zero(3, 1);
  }
  else
  {
    rotationAngle = M.norm();
    RotationAxis = M / rotationAngle;
  }

  Eigen::Matrix<T, 3, 3> WedgedMatrix = wedge_perator(RotationAxis);

  Eigen::Matrix<T, 3, 3> Result;
  Result = I + sin(rotationAngle) * WedgedMatrix + (1 - cos(rotationAngle)) * (WedgedMatrix * WedgedMatrix);

  return (Result);
}

template <typename T>
airsim_ros_pkgs::IntrinsicsCamera getInstrinscsMsg(T focal_length_in, T focus_distance_in, T aperture_in)
{
  airsim_ros_pkgs::IntrinsicsCamera msg;
  msg.focal_length = focal_length_in;
  msg.focus_distance = focus_distance_in;
  msg.aperture = aperture_in;
  return msg;
}

// J_im equations
template <typename T>
Pixel<T> calculate_image_position_from_3D(T focal_length_mm, T relative_position_x_m, T relative_position_y_m,
                                          T relative_position_z_m, Eigen::Matrix<T, 3, 3> R)
{
  // Read comment to understand change of variables
  T distance_y_mm = relative_position_y_m * 1000;  // ms to mms
  T distance_z_mm = relative_position_z_m * 1000;
  T distance_x_mm = relative_position_x_m * 1000;

  Eigen::Matrix<T, 3, 1> P(distance_x_mm, distance_y_mm, distance_z_mm);
  Eigen::Matrix<T, 3, 1> rPs = R.transpose() * P;

  Eigen::Matrix<T, 3, 1> vectorPixels(rPs(1), rPs(2), rPs(0));

  T x0_mm = sensor_width_mm / 2;
  T y0_mm = sensor_height_mm / 2;

  T x_mm = x0_mm + ((focal_length_mm * vectorPixels(0)) / vectorPixels(2));
  T y_mm = y0_mm + ((focal_length_mm * vectorPixels(1)) / vectorPixels(2));

  T xPixels_px = x_mm * picture_width_px / sensor_width_mm;

  T yPixels_px = y_mm * picture_height_px / sensor_height_mm;

  Pixel<T> p(xPixels_px, yPixels_px);

  return (p);
}

// J_DoF equations
template <typename T>
T calculate_hyperfocal_distance_DoF(T focal_length, T aperture)
{
  T div = 0.0;
  T term1 = (T)(pow(focal_length, 2) + 0.00001);
  T term2 = (T)((aperture * circle_confusion) + 0.00001);
  div = (T)term1 / (T)term2;

  return div + focal_length;
}

template <typename T>
T calculate_near_distance_DoF(T focus_distance, T hyperfocal_distance, T focal_length)
{
  T term1 = focus_distance * (hyperfocal_distance - focal_length);
  T term2 = hyperfocal_distance + focus_distance - 2 * focal_length;

  T div = (T)term1 / (T)term2;

  return (div);
}

template <typename T>
T calculate_far_distance_DoF(T focus_distance, T hyperfocal_distance, T focal_length)
{
  T term1 = focus_distance * (hyperfocal_distance - focal_length);
  T term2 = hyperfocal_distance - focus_distance;

  T div = (T)term1 / (T)term2;
  return (div);
}

std::string plot_values(cinempc::PlotValues plot_values, bool header)
{
  std::ostringstream result;
  if (header)
  {
    result << "Time"
           << ","
           << "Sequence"
           << ","
           << "Sim_time"
           << ","
           << "mpc_dt"
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
           << "v_x_gt"
           << ","
           << "v_y_gt"
           << ","
           << "v_z_gt"
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
    result << plot_values.time_ms << "," << plot_values.sequence << "," << sim_frequency << "," << mpc_dt << ","
           << plot_values.target_world_gt.x << "," << plot_values.target_world_gt.y << ","
           << plot_values.target_world_gt.z << "," << plot_values.target_world_perception.x << ","
           << plot_values.target_world_perception.y << "," << plot_values.target_world_perception.z << ","
           << plot_values.target_world_kf.x << "," << plot_values.target_world_kf.y << ","
           << plot_values.target_world_kf.z << "," << plot_values.mpc_plot_values.d_gt << ","
           << plot_values.v_target_kf.x << "," << plot_values.v_target_kf.y << "," << plot_values.v_target_kf.z << ","
           << plot_values.v_target_gt.x << "," << plot_values.v_target_gt.y << "," << plot_values.v_target_gt.z << ","
           << cinempc::quat_to_RPY<double>(plot_values.target_rot_gt).pitch << ","
           << cinempc::quat_to_RPY<double>(plot_values.target_rot_gt).yaw + target_1_yaw_gt << ","
           << cinempc::quat_to_RPY<double>(plot_values.target_rot_perception).pitch << ","
           << cinempc::quat_to_RPY<double>(plot_values.target_rot_perception).yaw << ","
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
           << cinempc::quat_to_RPY<double>(plot_values.constraints.targets_orientation_star.at(0)).roll << ","
           << cinempc::quat_to_RPY<double>(plot_values.constraints.targets_orientation_star.at(0)).pitch << ","
           << cinempc::quat_to_RPY<double>(plot_values.constraints.targets_orientation_star.at(0)).yaw << ","
           << plot_values.constraints.targets_d_star.at(0) << "," << plot_values.constraints.targets_im_top_star.at(0).x
           << "," << plot_values.constraints.targets_im_top_star.at(0).y << ","
           << plot_values.constraints.targets_im_center_star.at(0).y << ","
           << plot_values.constraints.targets_im_bottom_star.at(0).y << "," << plot_values.drone_position_gt.x << ","
           << plot_values.drone_position_gt.y << "," << plot_values.drone_position_gt.z << std::endl;
  }
  return result.str();
}
};  // namespace cinempc
