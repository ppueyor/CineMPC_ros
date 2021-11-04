#include "cinempc_main_node.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

// Current position of drone
std::vector<geometry_msgs::Pose> targets_poses;
float focal_length = 35, focus_distance = 10000, aperture = 20;

int index_splines = 0;
bool noise = true;
float sequence = 0;
int change_sequence_index = 0;
double steps_each_dt = 20;
double interval = dt / steps_each_dt;
double freq_loop = 1 / interval;

float focal_length_next_state = 35;
geometry_msgs::Pose drone_pose_next_state;

SimpleKalmanFilter targets_kalman_filters[] = {
  SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 0.05),
  SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 0.05)
};

std::vector<string> targets_names = { "Person1", "Person2" };

std::vector<double> times_vector, focal_length_vector, focus_distance_vector, aperture_vector, roll_vector, yaw_vector,
    pitch_vector;
tk::spline focal_length_spline, focus_distance_spline, aperture_spline, yaw_spline, pitch_spline;

static std::default_random_engine generator;
void myPoseMsgToTF(const geometry_msgs::Pose& msg, tf2::Transform& bt)
{
  bt = tf2::Transform(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                      tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
}

void changeSeqCallback(const std_msgs::Float32::ConstPtr& msg)
{
  sequence = msg->data;
}

double generateNoise(double mean, double st_dev)
{
  std::normal_distribution<double> distribution(mean, st_dev);
  double gaussian_noise = distribution(generator);

  // cout << "noise" << ": " << gaussian_noise << endl;
  if (noise)
  {
    return gaussian_noise;
  }
  else
  {
    return 0;
  }
}

cinempc::Constraints getConstraints(int sequence, std::vector<cinempc::TargetStates> targets,
                                    geometry_msgs::Pose drone_pose)
{
  cinempc::Constraints c;

  Eigen::Matrix<double, 3, 3> wRboy = cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw);
  Eigen::Matrix<double, 3, 3> wRw = cinempc::RPYtoRMatrix<double>(0, 0, PI);
  for (int i = 0; i < targets.size(); i++)
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

  sequence = 1;
  if (sequence == 1)
  {
    // starting boy. From front preseting boy focused and mid-body
    c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(0).poses_up.at(0).position.x,
                                                                targets.at(0).poses_up.at(0).position.y, 0, 0)) -
                1;
    c.weights.w_dn = 10 * 2;
    c.df_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(0).poses_up.at(0).position.x,
                                                                targets.at(0).poses_up.at(0).position.y, 0, 0)) +
                4;
    c.weights.w_df = 10 * 1;

    c.targets_im_up_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 10;
    ;                                                 // 10;                      // 1;                       // 1 * 1;
    c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 10;             // 10;                      // 1;                       // 1 * 1;
    c.targets_im_down_star.at(0).y = image_y_third_down + 60;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 0;  // 10;                      // 1;                       // 1 * 1;

    c.targets_im_up_star.at(1).x = -1;
    c.weights.w_img_targets.at(1).x = 0;
    c.targets_im_up_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(1).y = 0;
    c.targets_im_down_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(1).z = 0;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 1000;  // 1 * 1;
    cinempc::RPY<double> relative =
        cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw - PI / 2).transpose() * wRboy);
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 5000 * 3;

    c.targets_d_star.at(1) = -1;
    c.weights.w_d_targets.at(1) = 0;
    // c.p_girl.R = RPY(0, 0, 0);
    c.weights.w_R_targets.at(1) = 0;
  }
  else if (sequence == 2 || sequence == 2.5)
  {
    // Rotatind around boy. From right (90º) presenting boy focused and full-body. Boy centered
    float weight_y = 1;
    if (change_sequence_index <= 20)
    {
      weight_y = 0.05 * change_sequence_index;
      change_sequence_index++;
      cout << "-------------------------------------" << endl
           << "------    we:    " << weight_y << "       ------" << endl
           << "-------------------------------------" << endl;
    }
    // starting boy. From front preseting boy focused and mid-body
    c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(0).poses_up.at(0).position.x,
                                                                targets.at(0).poses_up.at(0).position.y, 0, 0)) -
                10;
    c.weights.w_dn = 10 * 2;
    c.df_star = 0;
    c.weights.w_df = 10 * 1;

    c.targets_im_up_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 1 * 2;
    c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 1 * 2.5;
    c.targets_im_down_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = weight_y;
    c.targets_im_up_star.at(1).x = 0;
    c.weights.w_img_targets.at(1).x = 0;
    c.targets_im_up_star.at(1).y = 0;
    c.weights.w_img_targets.at(1).y = 0;
    c.targets_im_down_star.at(1).y = -1;
    c.weights.w_img_targets.at(1).z = 0;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 100 * 1;
    cinempc::RPY<double> relative =
        cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, -0.3, subject_yaw - PI / 2).transpose() * wRboy);
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 5000 * 2;

    c.targets_d_star.at(1) = -1;
    c.weights.w_R_targets.at(0) = 0;
    // c.p_girl.R = RPY(0, 0, 0);
    c.weights.w_R_targets.at(1) = 0;
  }
  else if (sequence == 3)
  {
    // Zoom in boy. From behind (180º) presenting boy focused and mid-body. Boy centered
    c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(0).poses_up.at(0).position.x,
                                                                targets.at(0).poses_up.at(0).position.y, 0, 0)) -
                2.5;
    c.weights.w_dn = 10 * 5;
    c.df_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(0).poses_up.at(0).position.x,
                                                                targets.at(0).poses_up.at(0).position.y, 0, 0)) +
                1;
    c.weights.w_df = 10 * 5;

    c.targets_im_up_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 1 * 1;
    c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 1 * 2;
    c.targets_im_down_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 1 * 1;
    c.targets_im_up_star.at(1).x = 0;
    c.weights.w_img_targets.at(1).x = 0;
    c.targets_im_up_star.at(1).y = -1;
    c.weights.w_img_targets.at(1).y = 0;
    c.targets_im_down_star.at(1).y = -1;
    c.weights.w_img_targets.at(1).z = 0;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 1 * 2;
    cinempc::RPY<double> relative =
        cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw - PI).transpose() * wRboy);
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 5000 * 1;

    c.targets_d_star.at(1) = -1;
    c.weights.w_R_targets.at(0) = 0;
    // c.p_girl.R = RPY(0, 0, 0);
    c.weights.w_R_targets.at(1) = 0;
  }
  else if (sequence == 4)
  {
    // Presenting girl. From behind (180º) presenting boy and girl focused and full-body of boy. Boy and girl in thirds
    c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(0).poses_up.at(0).position.x,
                                                                targets.at(0).poses_up.at(0).position.y, 0, 0)) -
                5;
    c.weights.w_dn = 10 * 1;
    c.df_star = 0;
    c.weights.w_df = 0;

    c.targets_im_up_star.at(0).x = image_x_third_left;
    c.weights.w_img_targets.at(0).x = 1 * 1;
    c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 1 * 2;
    c.targets_im_down_star.at(0).y = image_y_third_down;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 1 * 1;
    c.targets_im_up_star.at(1).x = image_x_third_right;
    c.weights.w_img_targets.at(1).x = 1;
    c.targets_im_up_star.at(1).y = image_y_third_up;
    c.weights.w_img_targets.at(1).y = 1 * 2;
    c.targets_im_down_star.at(1).y = -1;
    c.weights.w_img_targets.at(1).z = 0;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 1 * 3;
    cinempc::RPY<double> relative = cinempc::RMatrixtoRPY<double>(
        cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw - PI / 2 - 0.15).transpose() * wRboy);
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 5000 * 0.5;

    c.targets_d_star.at(1) = -1;
    c.weights.w_R_targets.at(0) = 0;
    // c.p_girl.R = RPY(0, 0, 0);
    c.weights.w_R_targets.at(1) = 0;
  }
  else if (sequence == 5)
  {
    // Highligh girl. From behind (180º) presenting girl focused and boy not focused and full-body of girl. Boy and girl
    // in thirds
    c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(1).poses_up.at(0).position.x,
                                                                targets.at(1).poses_up.at(0).position.y, 0, 0)) -
                1;
    c.weights.w_dn = 10 * 1;
    c.df_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(1).poses_up.at(0).position.x,
                                                                targets.at(0).poses_up.at(0).position.y, 0, 0)) +
                20;
    c.weights.w_df = 5;

    c.targets_im_up_star.at(0).x = image_x_third_left;
    c.weights.w_img_targets.at(0).x = 1 * 1;
    c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 1 * 1;
    c.targets_im_down_star.at(0).y = 0;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 0;

    c.targets_im_up_star.at(1).x = image_x_third_right;
    c.weights.w_img_targets.at(1).x = 1 * 2;
    c.targets_im_up_star.at(1).y = image_y_third_up;
    c.weights.w_img_targets.at(1).y = 1 * 2;
    c.targets_im_down_star.at(1).y = image_y_third_down;
    c.weights.w_img_targets.at(1).z = 1;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 1 * 2;
    cinempc::RPY<double> relative = cinempc::RMatrixtoRPY<double>(
        cinempc::RPYtoRMatrix<double>(0, 0, subject_yaw - PI / 2 - 0.17).transpose() * wRboy);
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 5000 * 0.5;

    c.targets_d_star.at(1) = -1;
    c.weights.w_R_targets.at(0) = 0;
    // c.p_girl.R = RPY(0, 0, 0);
    c.weights.w_R_targets.at(1) = 0;
  }
  else if (sequence == 6)
  {
    // Focus just in girl. From front of girl (180º) focus on girl focused  focused and mid-body of girl centered. Girl
    c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(1).poses_up.at(0).position.x,
                                                                targets.at(1).poses_up.at(0).position.y, 0, 0)) -
                1;
    c.weights.w_dn = 10 * 1;
    c.df_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(1).poses_up.at(0).position.x,
                                                                targets.at(1).poses_up.at(0).position.y, 0, 0)) +
                35;
    c.weights.w_df = 10;

    c.targets_im_up_star.at(0).x = image_x_third_left;
    c.weights.w_img_targets.at(0).x = 0;
    c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 0;
    c.targets_im_down_star.at(0).y = 0;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 0;

    c.targets_im_up_star.at(1).x = image_x_center;
    c.weights.w_img_targets.at(1).x = 1 * 2;
    c.targets_im_up_star.at(1).y = image_y_third_up;
    c.weights.w_img_targets.at(1).y = 1 * 2;
    c.targets_im_down_star.at(1).y = image_y_third_down;
    c.weights.w_img_targets.at(1).z = 1;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 0;
    // c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 0;

    c.targets_d_star.at(1) = -1;
    c.weights.w_R_targets.at(0) = 0;
    cinempc::RPY<double> relative_girl =
        cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, -0.1, PI - PI - PI / 4).transpose() * wRw);
    tf2::Quaternion quaternion_tf2_girl;
    quaternion_tf2_girl.setRPY(relative_girl.roll, relative_girl.pitch, relative_girl.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2_girl);
    c.targets_orientation_star.at(1) = quaternion;
    c.weights.w_R_targets.at(1) = 10000;
  }
  else if (sequence == 7)
  {
    // Focus just in girl. From front of girl (180º) focus on girl focused  focused and mid-body of girl centered. Girl
    // centered Highligh girl. From behind (180º) presenting girl focused and boy not focused and full-body of girl. Boy

    c.dn_star = abs(cinempc::calculateDistanceTo2DPoint<double>(targets.at(1).poses_up.at(0).position.x,
                                                                targets.at(1).poses_up.at(0).position.y, 0, 0)) -
                5;
    c.weights.w_dn = 1;
    c.df_star = 0;
    c.weights.w_df = 0;

    c.targets_im_up_star.at(0).x = 0;
    c.weights.w_img_targets.at(0).x = 0;
    c.targets_im_up_star.at(0).y = 0;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 0;
    c.targets_im_down_star.at(0).y = 0;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 0;

    c.targets_im_up_star.at(1).x = image_x_center;
    c.weights.w_img_targets.at(1).x = 1 * 2;
    c.targets_im_up_star.at(1).y = image_y_third_up;
    c.weights.w_img_targets.at(1).y = 1;
    c.targets_im_down_star.at(1).y = image_y_third_down;
    c.weights.w_img_targets.at(1).z = 1;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 0;
    // c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 0;

    c.targets_d_star.at(1) = 5;
    c.weights.w_R_targets.at(0) = 1;

    cinempc::RPY<double> relative_girl =
        cinempc::RMatrixtoRPY<double>(cinempc::RPYtoRMatrix<double>(0, -0.3, PI - PI - 3 * PI / 4).transpose() * wRw);
    tf2::Quaternion quaternion_tf2_girl;
    quaternion_tf2_girl.setRPY(relative_girl.roll, relative_girl.pitch, relative_girl.yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2_girl);
    c.targets_orientation_star.at(1) = quaternion;
    c.weights.w_R_targets.at(1) = 10000;
  }
  return c;
}

void readDroneStateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  float drone_x = msg->pose.pose.position.x;
  float drone_y = msg->pose.pose.position.y;
  float drone_z = msg->pose.pose.position.z;

  drone_pose.position.x = drone_x;
  drone_pose.position.y = drone_y;
  drone_pose.position.z = drone_z;

  // drone_pose.orientation = msg->pose.pose.orientation;
}

void readTargetStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
{
  float target_x_noise = msg->pose.position.x;        // + generateNoise(0, 0.04);
  float target_y_noise = msg->pose.position.y;        // + generateNoise(0, 0.04);
  float target_z_noise = msg->pose.position.z - 1.8;  // + generateNoise(0, 0.04);

  float target_x = targets_kalman_filters[i * 0].updateEstimate(target_x_noise);
  float target_y = targets_kalman_filters[i * 1].updateEstimate(target_y_noise);
  float target_z = targets_kalman_filters[i * 2].updateEstimate(target_z_noise);

  // targets_poses.at(i).position.x = target_x;
  // targets_poses.at(i).position.y = target_y;
  // targets_poses.at(i).position.z = target_z;

  targets_poses.at(i).position.x = target_x_noise;
  targets_poses.at(i).position.y = target_y_noise;
  targets_poses.at(i).position.z = target_z_noise;

  targets_poses.at(i).orientation = msg->pose.orientation;
}

void initializeTargets()
{
  for (int i = 0; i < targets_names.size(); i++)
  {
    geometry_msgs::Pose p;
    targets_poses.push_back(p);
  }
}

void mpcResultCallback(const cinempc::MPCResult::ConstPtr& msg)
{
  std::vector<geometry_msgs::Point> pathMPC;
  focal_length_vector.clear();
  focus_distance_vector.clear();
  aperture_vector.clear();
  yaw_vector.clear();
  pitch_vector.clear();
  times_vector.clear();

  int index_mpc = 0;

  roll_vector.insert(roll_vector.begin(), cinempc::quatToRPY<double>(drone_pose.orientation).roll);
  pitch_vector.insert(pitch_vector.begin(), cinempc::quatToRPY<double>(drone_pose.orientation).pitch);
  yaw_vector.insert(yaw_vector.begin(), cinempc::quatToRPY<double>(drone_pose.orientation).yaw);
  focal_length_vector.insert(focal_length_vector.begin(), focal_length);
  focus_distance_vector.insert(focus_distance_vector.begin(), focus_distance);
  aperture_vector.insert(aperture_vector.begin(), aperture);
  times_vector.push_back(0);

  index_mpc++;

  double max_vel_x = 0, max_vel_y = 0, max_vel_z = 0;

  while (index_mpc < MPC_N)
  {
    cinempc::DroneAndCameraState cine_mpc_result = msg->mpc_n_states.at(index_mpc);
    times_vector.push_back(dt * index_mpc);

    focal_length_vector.insert(focal_length_vector.begin() + index_mpc, cine_mpc_result.intrinsics.focal_length);

    focus_distance_vector.insert(focus_distance_vector.begin() + index_mpc, cine_mpc_result.intrinsics.focus_distance);
    aperture_vector.insert(aperture_vector.begin() + index_mpc, cine_mpc_result.intrinsics.aperture);

    geometry_msgs::Pose world_T_result;

    world_T_result = cinempc::calculateDroneDistanceToWorld<double>(drone_pose, cine_mpc_result.drone_pose, false);

    cinempc::RPY<double> rpy = cinempc::quatToRPY<double>(world_T_result.orientation);
    yaw_vector.insert(yaw_vector.begin() + index_mpc, rpy.yaw);
    pitch_vector.insert(pitch_vector.begin() + index_mpc, rpy.pitch);

    geometry_msgs::Point path_point(world_T_result.position);
    pathMPC.push_back(path_point);

    std::cout << "Point:" << path_point.x << "  " << path_point.y << "   " << path_point.z << std::endl;

    if (index_mpc == 1)
    {
      focal_length_next_state = cine_mpc_result.intrinsics.focal_length;
      drone_pose_next_state = world_T_result;
    }

    geometry_msgs::Pose dTvel;
    dTvel.position = cine_mpc_result.velocity;
    dTvel.orientation = world_T_result.orientation;
    geometry_msgs::Pose wTvel = cinempc::calculateDroneDistanceToWorld<double>(drone_pose, dTvel, true);

    max_vel_x = max(abs(wTvel.position.x), max_vel_x);
    max_vel_y = max(abs(wTvel.position.y), max_vel_y);
    max_vel_z = max(abs(wTvel.position.z), max_vel_z);

    index_mpc++;
  }

  std::cout << "NEW POSE:" << cinempc::quatToRPY<double>(drone_pose.orientation).yaw << std::endl;

  for (double focal_l : focal_length_vector)
  {
    std::cout << "focal:" << focal_l << std::endl;
  }

  for (double focal_l : focus_distance_vector)
  {
    // std::cout << "focus:" << focal_l << std::endl;
  }

  for (double focal_l : aperture_vector)
  {
    // std::cout << "ap:" << focal_l << std::endl;
  }
  for (double focal_l : yaw_vector)
  {
    std::cout << "yaw:" << focal_l << std::endl;
  }
  focal_length_spline.set_points(times_vector, focal_length_vector);
  focus_distance_spline.set_points(times_vector, focus_distance_vector);
  aperture_spline.set_points(times_vector, aperture_vector);
  yaw_spline.set_points(times_vector, yaw_vector);
  pitch_spline.set_points(times_vector, pitch_vector);

  index_splines = 0;

  // move on path
  airsim_ros_pkgs::MoveOnPath srv;
  double max_vel = max(0.3, max(max_vel_x, max(max_vel_y, max_vel_z)));
  srv.request.vel = max_vel;
  srv.request.timeout = 10;
  srv.request.rads_yaw = cinempc::quatToRPY<double>(drone_pose.orientation).yaw;
  srv.request.positions = pathMPC;

  service_move_on_path.call(srv);
}

airsim_ros_pkgs::IntrinsicsCamera getInstrinscsMsg(float focal_length_in, float focus_distance_in, float aperture_in)
{
  airsim_ros_pkgs::IntrinsicsCamera msg;
  msg.focal_length = focal_length_in;
  msg.focus_distance = focus_distance_in;
  msg.aperture = aperture_in;
  return msg;
}

void publishNewStateToMPC(const ros::TimerEvent& e, ros::NodeHandle n)
{
  cinempc::MPCIncomingState msg;
  msg.drone_state.drone_pose.position.x = 0;  // drone_pose.position.x;
  msg.drone_state.drone_pose.position.y = 0;  // drone_pose.position.y;
  msg.drone_state.drone_pose.position.z = 0;  // drone_pose.position.z;

  geometry_msgs::Quaternion q = cinempc::RPYToQuat(0, 0, 0);
  msg.drone_state.drone_pose.orientation = q;  // drone_pose.orientation;

  msg.floor_pos = -drone_pose.position.z;

  msg.drone_state.intrinsics = getInstrinscsMsg(focal_length_next_state, focus_distance, aperture);

  Eigen::VectorXd xvals(30);
  Eigen::VectorXd yvals(30);

  for (int i = 0; i < targets_names.size(); i++)
  {
    cinempc::TargetStates states;
    msg.targets.push_back(states);
  }
  int targets = 0;
  for (int j = 0; j < targets_names.size(); j++)
  {
    ros::ServiceClient service_get_target_poses =
        n.serviceClient<cinempc::GetNextPersonPoses>("cinempc/" + targets_names.at(j) + "/get_next_poses");
    cinempc::GetNextPersonPoses srv;

    //  std::cout << "yaw:" << quatToRPY(drone_pose.orientation).yaw << std::endl;

    srv.request.current_pose = targets_poses.at(j);

    if (service_get_target_poses.call(srv))
    {
      std::vector<geometry_msgs::Pose> vector;
      geometry_msgs::PoseArray next_target_steps = srv.response.pose_array;
      cinempc::TargetStates target_state = {};
      for (int i = 0; i < MPC_N * 2; i++)
      {
        geometry_msgs::Pose p =
            cinempc::calculate_relative_pose_drone_person<double>(next_target_steps.poses.at(i), drone_pose_next_state);
        geometry_msgs::Pose p2 = next_target_steps.poses.at(i);

        if (i < MPC_N)
        {
          target_state.poses_up.push_back(p);
        }
        else
        {
          target_state.poses_down.push_back(p);
        }
      }
      target_state.target_name = targets_names.at(j);
      msg.targets.at(j) = target_state;
      targets++;
      if (targets == targets_names.size())
      {
        msg.constraints = getConstraints(sequence, msg.targets, drone_pose_next_state);
        new_state_publisher.publish(msg);
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_main");

  initializeTargets();

  ros::NodeHandle n;

  ros::ServiceClient service_take_off = n.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
  airsim_ros_pkgs::Takeoff srv;
  srv.request.waitOnLastTask = false;

  // service_take_off.call(srv);

  ros::Subscriber change_sequence_sub = n.subscribe("cinempc/sequence", 1000, changeSeqCallback);

  ros::Subscriber new_drone_state_received_sub = n.subscribe("airsim_node/drone_1/"
                                                             "odom_local_ned",
                                                             1000, readDroneStateCallback);

  ros::Subscriber mpc_result_n_steps_sub = n.subscribe("cinempc/next_n_states", 1000, mpcResultCallback);

  std::vector<ros::Subscriber> targets_states_subscribers = {};
  for (int i = 0; i < targets_names.size(); i++)
  {
    targets_states_subscribers.push_back(n.subscribe<geometry_msgs::PoseStamped>(
        "airsim_node/" + targets_names.at(i) + "/get_pose", 1000, boost::bind(&readTargetStateCallback, _1, i)));
  }

  intrinsics_publisher = n.advertise<airsim_ros_pkgs::IntrinsicsCamera>("/airsim_node/drone_1/set_intrinsics", 10);

  intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance, aperture));

  gimbal_rotation_publisher =
      n.advertise<airsim_ros_pkgs::GimbalAngleQuatCmd>("/airsim_node/gimbal_angle_quat_cmd", 10);

  new_state_publisher = n.advertise<cinempc::MPCIncomingState>("cinempc/current_state", 10);

  cinempc_calculate_new_states_timer_ = n.createTimer(ros::Duration(dt), boost::bind(publishNewStateToMPC, _1, n));

  service_move_on_path = n.serviceClient<airsim_ros_pkgs::MoveOnPath>("/airsim_node/drone_1/move_on_path");

  // init camera pose
  airsim_ros_pkgs::GimbalAngleQuatCmd msg;
  geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, 0, drone_start_yaw);
  msg.orientation = q;
  drone_pose.orientation = q;
  gimbal_rotation_publisher.publish(msg);

  ros::Rate loop_rate(freq_loop);
  while (ros::ok())
  {
    if (focal_length_spline.get_x().size() != 0)
    {
      focal_length = focal_length_spline(interval * index_splines);
      focus_distance = focus_distance_spline(interval * index_splines);
      aperture = aperture_spline(interval * index_splines);

      airsim_ros_pkgs::IntrinsicsCamera intrinsics_msg;
      intrinsics_msg.focal_length = focal_length;
      intrinsics_msg.focus_distance = focus_distance * 100;
      intrinsics_msg.aperture = aperture;
      intrinsics_publisher.publish(intrinsics_msg);

      double yaw_gimbal = yaw_spline(interval * index_splines);
      double pitch_gimbal = pitch_spline(interval * index_splines);
      geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, pitch_gimbal, yaw_gimbal);
      // std::cout << "yaw1:" << quatToRPY(drone_pose.orientation).yaw << std::endl;

      airsim_ros_pkgs::GimbalAngleQuatCmd msg;
      msg.orientation = q;
      drone_pose.orientation = q;
      // std::cout << "focal_length:" << focal_length << " index: " << index_splines
      //         << std::endl;  // quatToRPY(msg.orientation).yaw
      gimbal_rotation_publisher.publish(msg);
    }

    index_splines++;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
