#include "cinempc_main_node.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

// Current position of drone
geometry_msgs::Pose drone_pose;
std::vector<geometry_msgs::Pose> targets_poses;
float focal_length = 35, focus_distance = 10000, aperture = 20;
int index_splines = 0;
bool noise = true;
float sequence = 0;
int change_sequence_index = 0;

SimpleKalmanFilter targets_kalman_filters[] = {
  SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 0.05),
  SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 0.05)
};

std::vector<string> targets_names = { "Person1", "Person2" };

std::vector<double> times_vector, focal_length_vector, focus_distance_vector, aperture_vector, roll_vector, yaw_vector,
    pitch_vector;
tk::spline focal_length_spline;

static std::default_random_engine generator;
void myPoseMsgToTF(const geometry_msgs::Pose& msg, tf2::Transform& bt)
{
  bt = tf2::Transform(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                      tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
}
geometry_msgs::Pose calculateRelativeDistanceDronePerson(geometry_msgs::Pose person_pose)
{
  tf2::Transform drone_pose_tf;
  tf2::Transform person_pose_tf;
  myPoseMsgToTF(drone_pose, drone_pose_tf);
  myPoseMsgToTF(person_pose, person_pose_tf);

  tf2::Transform relative_position = drone_pose_tf.inverseTimes(person_pose_tf);

  geometry_msgs::Pose p;
  p.position.x = relative_position.getOrigin().getX();
  p.position.y = relative_position.getOrigin().getY();
  p.position.z = relative_position.getOrigin().getZ();

  p.orientation = tf2::toMsg(relative_position.getRotation());

  return (p);
}

RPY quatToRPY(geometry_msgs::Quaternion q)
{
  double roll, pitch, yaw;
  tf2::Quaternion quat_tf;
  tf2::fromMsg(q, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
  RPY rpy;
  rpy.roll = roll;
  rpy.pitch = pitch;
  rpy.yaw = yaw;
  return rpy;
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

// Function to calculate distance between two points
double calculateEucDistance(double x1, double y1, double x2, double y2)
{
  // Calculating distance
  return (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0));
}

Eigen::Matrix<double, 3, 3> RPYtoRMatrix(double roll, double pitch, double yaw)
{
  // Eigen::Quaterniond quat = VectorMath::toQuaternion(0,0,CppAD::Value(yaw)).Quaternion;
  Eigen::AngleAxis<double> rollAngle(roll, Eigen::Matrix<double, 1, 3>::UnitX());
  Eigen::AngleAxis<double> pitchAngle(pitch, Eigen::Matrix<double, 1, 3>::UnitY());
  Eigen::AngleAxis<double> yawAngle(yaw, Eigen::Matrix<double, 1, 3>::UnitZ());

  Eigen::Matrix<double, 3, 3> R;

  Eigen::Quaternion<double> q = yawAngle * rollAngle * pitchAngle;

  R = q.matrix();

  return (R);
}

RPY RMatrixtoRPY(Eigen::Matrix<double, 3, 3> R)
{
  double roll = asin(R(2, 1));
  double pitch = -atan2(R(2, 0), R(2, 2));
  double yaw = -atan2(R(0, 1), R(1, 1));

  RPY RPY;
  RPY.roll = roll;
  RPY.pitch = pitch;
  RPY.yaw = yaw;

  return RPY;
}

cinempc::Constraints getConstraints(int sequence, std::vector<cinempc::TargetStates> targets,
                                    geometry_msgs::Pose drone_pose)
{
  cinempc::Constraints c;

  Eigen::Matrix<double, 3, 3> wRboy = RPYtoRMatrix(0, 0, subject_yaw);
  Eigen::Matrix<double, 3, 3> wRw = RPYtoRMatrix(0, 0, PI);
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
    c.dn_star = abs(calculateEucDistance(targets.at(0).poses_up.at(0).position.x,
                                         targets.at(0).poses_up.at(0).position.y, 0, 0)) -
                1;
    c.weights.w_dn = 10 * 2;
    c.df_star = abs(calculateEucDistance(targets.at(0).poses_up.at(0).position.x,
                                         targets.at(0).poses_up.at(0).position.y, 0, 0)) +
                4;
    c.weights.w_df = 10 * 1;

    c.targets_im_up_star.at(0).x = image_x_center;
    c.weights.w_img_targets.at(0).x = 1 * 1;
    c.targets_im_up_star.at(0).y = image_y_third_up;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).y = 1 * 1;
    c.targets_im_down_star.at(0).y = image_y_third_down + 60;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(0).z = 1 * 1;

    c.targets_im_up_star.at(1).x = -1;
    c.weights.w_img_targets.at(1).x = 0;
    c.targets_im_up_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(1).y = 0;
    c.targets_im_down_star.at(1).y = -1;  // mid-body (control with calculations of positions)
    c.weights.w_img_targets.at(1).z = 0;

    c.targets_d_star.at(0) = 5;
    c.weights.w_d_targets.at(0) = 1 * 1;
    RPY relative = RMatrixtoRPY(RPYtoRMatrix(0, 0, subject_yaw - PI + 0.1).transpose() * wRboy);
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(relative.roll, relative.pitch, relative.yaw);
    cout << "-------------------------------------" << endl
         << "------    we:    " << relative.pitch << relative.roll << relative.yaw << "       ------" << endl
         << "-------------------------------------" << endl;
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    c.targets_orientation_star.at(0) = quaternion;
    c.weights.w_R_targets.at(0) = 5000 * 0.5;

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
    c.dn_star = abs(calculateEucDistance(targets.at(0).poses_up.at(0).position.x,
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
    c.weights.w_d_targets.at(0) = 1 * 1;
    RPY relative = RMatrixtoRPY(RPYtoRMatrix(0, -0.3, subject_yaw - PI / 2).transpose() * wRboy);
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
    c.dn_star = abs(calculateEucDistance(targets.at(0).poses_up.at(0).position.x,
                                         targets.at(0).poses_up.at(0).position.y, 0, 0)) -
                2.5;
    c.weights.w_dn = 10 * 5;
    c.df_star = abs(calculateEucDistance(targets.at(0).poses_up.at(0).position.x,
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
    RPY relative = RMatrixtoRPY(RPYtoRMatrix(0, 0, subject_yaw - PI).transpose() * wRboy);
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
    c.dn_star = abs(calculateEucDistance(targets.at(0).poses_up.at(0).position.x,
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
    RPY relative = RMatrixtoRPY(RPYtoRMatrix(0, 0, subject_yaw - PI / 2 - 0.15).transpose() * wRboy);
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
    c.dn_star = abs(calculateEucDistance(targets.at(1).poses_up.at(0).position.x,
                                         targets.at(1).poses_up.at(0).position.y, 0, 0)) -
                1;
    c.weights.w_dn = 10 * 1;
    c.df_star = abs(calculateEucDistance(targets.at(1).poses_up.at(0).position.x,
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
    RPY relative = RMatrixtoRPY(RPYtoRMatrix(0, 0, subject_yaw - PI / 2 - 0.17).transpose() * wRboy);
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
    c.dn_star = abs(calculateEucDistance(targets.at(1).poses_up.at(0).position.x,
                                         targets.at(1).poses_up.at(0).position.y, 0, 0)) -
                1;
    c.weights.w_dn = 10 * 1;
    c.df_star = abs(calculateEucDistance(targets.at(1).poses_up.at(0).position.x,
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
    RPY relative_girl = RMatrixtoRPY(RPYtoRMatrix(0, -0.1, PI - PI - PI / 4).transpose() * wRw);
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

    c.dn_star = abs(calculateEucDistance(targets.at(1).poses_up.at(0).position.x,
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
    RPY relative_girl = RMatrixtoRPY(RPYtoRMatrix(0, -0.3, PI - PI - 3 * PI / 4).transpose() * wRw);
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

  drone_pose.orientation = msg->pose.pose.orientation;
}

void readTargetStateCallback(const geometry_msgs::Pose::ConstPtr& msg, int i)
{
  float target_x_noise = msg->position.x + generateNoise(0, 0.04);
  float target_y_noise = msg->position.y + generateNoise(0, 0.04);
  float target_z_noise = msg->position.z + generateNoise(0, 0.04);

  float target_x = targets_kalman_filters[i * 0].updateEstimate(target_x_noise);
  float target_y = targets_kalman_filters[i * 1].updateEstimate(target_y_noise);
  float target_z = targets_kalman_filters[i * 2].updateEstimate(target_z_noise);

  targets_poses.at(i).position.x = target_x;
  targets_poses.at(i).position.y = target_y;
  targets_poses.at(i).position.z = target_z;
}

void initializeTargets()
{
  for (int i = 0; i < targets_names.size(); i++)
  {
    geometry_msgs::Pose p;
    targets_poses.push_back(p);
  }
}

void calculateNextNStatesCallback(const cinempc::MPCResult::ConstPtr& msg)
{
  focal_length_vector.clear();
  index_splines = 0;
  int index_mpc = 0;
  focal_length_vector.insert(focal_length_vector.begin() + index_mpc, focal_length);
  while (index_mpc < MPC_N)
  {
    airsim_ros_pkgs::DroneAndCameraState current_state = msg->mpc_n_states.at(index_mpc);
    times_vector.push_back(dt * index_mpc);
    focal_length_vector.insert(focal_length_vector.begin() + index_mpc, current_state.instrinsics.focal_length);
    index_mpc++;
  }

  focal_length_spline.set_points(times_vector, focal_length_vector);
}

airsim_ros_pkgs::IntrinsicsCamera getInstrinscsMsg(float focal_length_in, float focus_distance_in, float aperture_in)
{
  airsim_ros_pkgs::IntrinsicsCamera msg;
  msg.focal_length = focal_length_in;
  msg.focus_distance = focus_distance_in;
  msg.aperture = aperture_in;
  return msg;
}

// thread to read the position of the drone
void publishNewStateToMPC(const ros::TimerEvent& e, ros::NodeHandle n)
{
  cinempc::MPCIncomingState msg;
  msg.drone_state.drone_pose.position.x = drone_pose.position.x;
  msg.drone_state.drone_pose.position.y = drone_pose.position.y;
  msg.drone_state.drone_pose.position.z = drone_pose.position.z;

  msg.drone_state.drone_pose.orientation = drone_pose.orientation;

  msg.floor_pos = -drone_pose.position.z;

  msg.drone_state.instrinsics = getInstrinscsMsg(focal_length, focus_distance, aperture);

  Eigen::VectorXd xvals(30);
  Eigen::VectorXd yvals(30);

  for (int i = 0; i < targets_names.size(); i++)
  {
    cinempc::TargetStates states;
    msg.targets.push_back(states);
  }
  for (int i = 0; i < targets_names.size(); i++)
  {
    ros::ServiceClient service_get_target_poses =
        n.serviceClient<cinempc::GetNextPersonPoses>("cinempc/" + targets_names.at(i) + "/get_next_poses");
    cinempc::GetNextPersonPoses srv;

    srv.request.current_pose = targets_poses.at(i);

    if (service_get_target_poses.call(srv))
    {
      std::vector<geometry_msgs::Pose> vector;
      geometry_msgs::PoseArray next_target_steps = srv.response.pose_array;
      cinempc::TargetStates target_state = {};
      for (int i = 0; i < MPC_N * 2; i++)
      {
        geometry_msgs::Pose p = next_target_steps.poses.at(i);
        if (i < MPC_N)
        {
          target_state.poses_up.push_back(p);
        }
        else
        {
          target_state.poses_down.push_back(p);
        }
      }
      target_state.target_name = targets_names.at(i);
      msg.targets.at(i) = target_state;
    }
  }

  msg.constraints = getConstraints(sequence, msg.targets, msg.drone_state.drone_pose);

  new_state_publisher.publish(msg);
  int a = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_main");

  initializeTargets();

  ros::NodeHandle n;
  ros::Subscriber change_sequence_sub = n.subscribe("cinempc/sequence", 1000, changeSeqCallback);

  ros::Subscriber new_drone_state_received_sub = n.subscribe("airsim_node/drone_1/"
                                                             "odom_local_ned",
                                                             1000, readDroneStateCallback);
  // std::vector<ros::Subscriber<geometry_msgs::PoseStamped>> targets_states_subscribers = {};
  for (int i = 0; i < targets_names.size(); i++)
  {
    n.subscribe<geometry_msgs::Pose>(targets_names.at(i) + "/get_pose", 1000,
                                     boost::bind(&readTargetStateCallback, _1, i));
  }

  intrinsics_publisher = n.advertise<airsim_ros_pkgs::IntrinsicsCamera>("drone_1/intrinsics", 10);

  intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance, aperture));

  new_state_publisher = n.advertise<cinempc::MPCIncomingState>("cinempc/current_state", 10);

  cinempc_calculate_new_states_timer_ = n.createTimer(ros::Duration(0.4), boost::bind(publishNewStateToMPC, _1, n));

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    // float focal_length = focal_length_spline(index_splines);
    // airsim_ros_pkgs::IntrinsicsCamera intrinsics_msg;
    // intrinsics_msg.focal_length = focal_length;
    // intrinsics_publisher.publish(intrinsics_msg);

    index_splines++;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
