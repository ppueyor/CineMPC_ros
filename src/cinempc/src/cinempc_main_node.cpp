#include "cinempc_main_node.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

// Current position of drone
std::vector<cinempc::PersonStatePerception> targets_states;
std::vector<geometry_msgs::Pose> targets_poses1;
string errors;

float focal_length = 35, focus_distance = 10000, aperture = 20;

int index_splines = 0;
bool noise = true;
float sequence = 1;
int change_sequence_index = 0;
double steps_each_dt = 20;
double interval = dt / steps_each_dt;
double freq_loop = 1 / interval;
bool stop = false;

ros::Time start_log;

std::stringstream logErrorFileName;
std::ofstream errorFile;

cinempc::PerceptionMsg perception_msg;

float focal_length_next_state = 35;
geometry_msgs::Pose drone_pose_next_state;

KalmanFilterEigen kalman_filter_targets;

SimpleKalmanFilter targets_kalman_filters[] = {
  SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 0.05),
  SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 1), SimpleKalmanFilter(0.04, 0.01, 0.05)
};

std::vector<double> times_vector, focal_length_vector, focus_distance_vector, aperture_vector, roll_vector, yaw_vector,
    pitch_vector;
tk::spline focal_length_spline, focus_distance_spline, aperture_spline, yaw_spline, pitch_spline;

static std::default_random_engine generator;
void myPoseMsgToTF(const geometry_msgs::Pose& msg, tf2::Transform& bt)
{
  bt = tf2::Transform(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                      tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
}

void initializePerceptionMsg()
{
  perception_msg.depth = sensor_msgs::Image();
  perception_msg.rgb = sensor_msgs::Image();
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
void initializeKalmanFilter()
{
  int n = 6;  // Number of states
  int m = 3;  // Number of measurements

  double dt = 0.2;  // Time step

  Eigen::MatrixXd A(n, n);  // System dynamics matrix
  Eigen::MatrixXd C(m, n);  // Output matrix
  Eigen::MatrixXd Q(n, n);  // Process noise covariance
  Eigen::MatrixXd R(m, m);  // Measurement noise covariance
  Eigen::MatrixXd P(n, n);  // Estimate error covariance

  A << 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  C << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  // Reasonable covariance matrices
  Q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  R << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;

  P << 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0,
      0, 0.5;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilterEigen init(dt, A, C, Q, R, P, n, m);

  Eigen::VectorXd x0(n);
  double t = 0;
  x0 << 0, 0, 0, 0, 0, 0;
  init.init(t, x0);

  kalman_filter_targets = init;
}
std::vector<geometry_msgs::Point> updateKalmanWithNewMeasure(geometry_msgs::Pose world_pose_target)
{
  Eigen::VectorXd measurement(kalman_filter_targets.numberOfMeasurements());
  measurement(0) = world_pose_target.position.x;
  measurement(1) = world_pose_target.position.y;
  measurement(2) = world_pose_target.position.z;
  kalman_filter_targets.update(measurement);

  Eigen::VectorXd new_state(kalman_filter_targets.numberOfStates());

  new_state = kalman_filter_targets.state();

  geometry_msgs::Point new_position;
  geometry_msgs::Point new_velocity;

  new_position.x = new_state(0);
  new_position.y = new_state(1);
  new_position.z = new_state(2);

  new_velocity.x = new_state(3);
  new_velocity.y = new_state(4);
  new_velocity.z = new_state(5);

  std::vector<geometry_msgs::Point> vector;
  vector.push_back(new_position);
  vector.push_back(new_velocity);

  return vector;
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
  float target_x_noise = msg->pose.position.x;         // + generateNoise(0, 0.04);
  float target_y_noise = msg->pose.position.y;         // + generateNoise(0, 0.04);
  float target_z_noise = msg->pose.position.z - 2.04;  // + generateNoise(0, 0.04);

  float target_x = targets_kalman_filters[i * 0].updateEstimate(target_x_noise);
  float target_y = targets_kalman_filters[i * 1].updateEstimate(target_y_noise);
  float target_z = targets_kalman_filters[i * 2].updateEstimate(target_z_noise);

  // targets_poses.at(i).position.x = target_x;
  // targets_poses.at(i).position.y = target_y;
  // targets_poses.at(i).position.z = target_z;

  targets_poses1.at(i).position.x = target_x_noise;
  targets_poses1.at(i).position.y = target_y_noise;
  targets_poses1.at(i).position.z = target_z_noise;

  targets_poses1.at(i).orientation = msg->pose.orientation;
}

void readTargetStatePerceptionCallback(const cinempc::PersonStatePerception::ConstPtr& msg, int target_index)
{
  // For now, msg just contains positions
  geometry_msgs::Pose pose_head, pose_hips, pose_feet;

  // relative position drone_target
  pose_head.position = msg->pose_head.position;
  pose_hips.position = msg->pose_hips.position;
  pose_feet.position = msg->pose_feet.position;

  // world position target
  geometry_msgs::Pose wTt = cinempc::calculate_world_pose_from_relative<double>(drone_pose, msg->pose_head, false);

  std::vector<geometry_msgs::Point> state = updateKalmanWithNewMeasure(wTt);
  geometry_msgs::Pose p_gt = targets_poses1.at(0);
  // std::cout << "gt_x: " << p_gt.position.x << "  gt_y: " << p_gt.position.y << "  gt_z " << p_gt.position.z <<
  // std::endl
  //           << std::endl
  //           << "wvt_x: " << wTt.position.x << "  wvt_y: " << wTt.position.y << "  wvt_z " << wTt.position.z <<
  //           std::endl
  //           << std::endl
  //           << "est_x: " << state.at(0).x << "  est_y: " << state.at(0).y << "  est_z: " << state.at(0).z <<
  //           std::endl
  //           << std::endl
  //           << "est_vx: " << state.at(1).x << "  est_vy: " << state.at(1).y << "  est_vz: " << state.at(1).z;

  Eigen::Matrix<double, 3, 1> wvt(state.at(1).x, state.at(1).y, state.at(1).z);
  wvt.normalize();
  Eigen::Matrix<double, 3, 1> g(0, 0, 9.8);
  g.normalize();
  Eigen::Matrix<double, 3, 1> a = g.cross(wvt);
  a.normalize();
  Eigen::Matrix<double, 3, 1> b = wvt.cross(a);
  b.normalize();

  // Now we have R in the world, we suppose all the parts of the target will have the same orientation
  Eigen::Matrix<double, 3, 3> R;
  R.col(0) = wvt;
  R.col(1) = a;
  R.col(2) = b;

  wTt.orientation = cinempc::RMatrixToQuat<double>(R);
  geometry_msgs::Quaternion relative_orientation =
      cinempc::calculate_relative_pose_drone_person<double>(wTt, drone_pose).orientation;

  // Needs relative! This is world
  pose_head.orientation = relative_orientation;
  pose_hips.orientation = relative_orientation;
  pose_feet.orientation = relative_orientation;

  targets_states.at(target_index).pose_head = pose_head;
  targets_states.at(target_index).pose_hips = pose_hips;
  targets_states.at(target_index).pose_feet = pose_feet;

  cinempc::RPY<double> rpy = cinempc::RMatrixtoRPY<double>(R);
  cinempc::RPY<double> rpy_d = cinempc::RMatrixtoRPY<double>(cinempc::quatToRMatrix<double>(drone_pose.orientation));
}

void initializeTargets()
{
  for (int i = 0; i < targets_names.size(); i++)
  {
    geometry_msgs::Pose p;
    targets_poses1.push_back(p);

    cinempc::PersonStatePerception state;
    targets_states.push_back(state);
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

    world_T_result = cinempc::calculate_world_pose_from_relative<double>(drone_pose, cine_mpc_result.drone_pose);

    cinempc::RPY<double> rpy = cinempc::quatToRPY<double>(world_T_result.orientation);
    cinempc::RPY<double> rpy_drone = cinempc::quatToRPY<double>(cine_mpc_result.drone_pose.orientation);
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
    geometry_msgs::Pose wTvel = cinempc::calculate_world_pose_from_relative<double>(drone_pose, dTvel, true);

    max_vel_x = max(abs(wTvel.position.x), max_vel_x);
    max_vel_y = max(abs(wTvel.position.y), max_vel_y);
    max_vel_z = max(abs(wTvel.position.z), max_vel_z);

    index_mpc++;
  }

  // std::cout << "NEW POSE:" << cinempc::quatToRPY<double>(drone_pose.orientation).yaw << std::endl;

  for (double focal_l : focal_length_vector)
  {
    // std::cout << "focal:" << focal_l << std::endl;
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
    // std::cout << "yaw:" << focal_l << std::endl;
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
  cinempc::MPCIncomingState msg_perception, msg_gt;
  msg_perception.drone_state.drone_pose.position.x = 0;  // drone_pose.position.x;
  msg_perception.drone_state.drone_pose.position.y = 0;  // drone_pose.position.y;
  msg_perception.drone_state.drone_pose.position.z = 0;  // drone_pose.position.z;

  geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, 0, 0);
  msg_perception.drone_state.drone_pose.orientation = q;  // drone_pose.orientation;

  msg_perception.floor_pos = -drone_pose.position.z;

  msg_perception.drone_state.intrinsics = getInstrinscsMsg(focal_length_next_state, focus_distance, aperture);

  msg_gt.drone_state.drone_pose.position.x = 0;   // drone_pose.position.x;
  msg_gt.drone_state.drone_pose.position.y = 0;   // drone_pose.position.y;
  msg_gt.drone_state.drone_pose.position.z = 0;   // drone_pose.position.z;
  msg_gt.drone_state.drone_pose.orientation = q;  // drone_pose.orientation;
  msg_gt.floor_pos = -drone_pose.position.z;

  msg_gt.drone_state.intrinsics = getInstrinscsMsg(focal_length_next_state, focus_distance, aperture);

  for (int i = 0; i < targets_names.size(); i++)
  {
    cinempc::PersonStateMPC states;
    msg_perception.targets.push_back(states);
    msg_gt.targets.push_back(states);
  }
  int targets = 0;
  for (int j = 0; j < targets_names.size(); j++)
  {
    cinempc::PersonStatePerception person_state = targets_states.at(j);
    cinempc::PersonStateMPC target_state = {};
    for (int i = 0; i < MPC_N * 2; i++)
    {
      // up part
      if (i < MPC_N)
      {
        target_state.poses_up.push_back(person_state.pose_head);
      }
      else
      {
        if (sequence == 1 || sequence == 3)
        {
          target_state.poses_down.push_back(person_state.pose_hips);
        }
        else
        {
          target_state.poses_down.push_back(person_state.pose_feet);
        }
      }
    }
    geometry_msgs::Pose p_gt = cinempc::calculate_relative_pose_drone_person<double>(targets_poses1.at(j), drone_pose);

    if (j == 0)
    {
      float error_x = abs(person_state.pose_head.position.x - (p_gt.position.x));
      float error_y = abs(person_state.pose_head.position.y - (p_gt.position.y));
      float error_z = abs(person_state.pose_head.position.z - (p_gt.position.z));

      // std::cout << "perception_x: " << person_state.pose_head.position.x
      //           << "  perception_y: " << person_state.pose_head.position.y
      //           << "  perception_z: " << person_state.pose_head.position.z << std::endl;

      // std::cout << "x_ground: " << p_gt.position.x << "  y_ground: " << p_gt.position.y
      //           << "  z_ground: " << p_gt.position.z << std::endl;
      // std::cout << "error_x: " << error_x << "  error_y: " << error_y << "  error_z: " << error_z << std::endl
      //           << std::endl;

      ros::Time end_log = ros::Time::now();
      ros::Duration diff = end_log - start_log;

      errorFile << diff.toNSec() / 1000000 << "," << p_gt.position.x << "," << p_gt.position.y << "," << p_gt.position.z
                << "," << person_state.pose_head.position.x << "," << person_state.pose_head.position.y << ","
                << person_state.pose_head.position.z << "," << error_x << "," << error_y << "," << error_z << std::endl;
    }
    target_state.target_name = targets_names.at(j);
    msg_perception.targets.at(j) = target_state;

    targets++;
    // targets++;
    if (targets == targets_names.size())
    {
      ros::ServiceClient service_get_user_constraints = n.serviceClient<cinempc::GetUserConstraints>("/cinempc/"
                                                                                                     "user_node/"
                                                                                                     "get_"
                                                                                                     "constraints");
      cinempc::GetUserConstraints srv;
      srv.request.targets_relative = msg_perception.targets;
      srv.request.drone_pose = drone_pose;
      cinempc::RPY<double> rpy = cinempc::quatToRPY<double>(msg_perception.targets.at(0).poses_up.at(0).orientation);
      srv.request.sequence = 1;

      if (service_get_user_constraints.call(srv))
      {
        msg_perception.constraints = srv.response.contraints;
        new_MPC_state_publisher.publish(msg_perception);
        // std::cout << "PUBLISHED" << std::endl;
      }
    }

    ros::ServiceClient service_get_target_poses =
        n.serviceClient<cinempc::GetNextPersonPoses>("cinempc/" + targets_names.at(j) + "/get_next_poses");
    cinempc::GetNextPersonPoses srv;

    //  std::cout << "yaw:" << quatToRPY(drone_pose.orientation).yaw << std::endl;

    srv.request.current_pose = targets_poses1.at(j);

    if (service_get_target_poses.call(srv))
    {
      std::vector<geometry_msgs::Pose> vector;
      geometry_msgs::PoseArray next_target_steps = srv.response.pose_array;
      cinempc::PersonStateMPC target_state1 = {};
      for (int i = 0; i < MPC_N * 2; i++)
      {
        geometry_msgs::Pose p =
            cinempc::calculate_relative_pose_drone_person<double>(next_target_steps.poses.at(i), drone_pose);
        geometry_msgs::Pose p2 = next_target_steps.poses.at(i);

        if (i < MPC_N)
        {
          target_state1.poses_up.push_back(p);
        }
        else
        {
          target_state1.poses_down.push_back(p);
        }
      }
      target_state1.target_name = targets_names.at(j);
      msg_gt.targets.at(j) = target_state1;
      // targets++;
      if (targets == targets_names.size())
      {
        ros::ServiceClient service_get_user_constraints = n.serviceClient<cinempc::GetUserConstraints>("/cinempc/"
                                                                                                       "user_node/"
                                                                                                       "get_"
                                                                                                       "constraints");
        cinempc::GetUserConstraints srv;
        srv.request.targets_relative = msg_gt.targets;
        srv.request.sequence = 1;

        // if (service_get_user_constraints.call(srv))
        //{
        // msg_gt.constraints = srv.response.contraints;
        // new_MPC_state_publisher.publish(msg_gt);
        //}
      }
    }
  }
}
void rgbReceivedCallback(const sensor_msgs::Image& msg)
{
  perception_msg.rgb = msg;
  if (perception_msg.depth.height != 0)
  {
    perception_msg.drone_state.drone_pose = drone_pose;
    perception_msg.drone_state.intrinsics = getInstrinscsMsg(focal_length, focus_distance, aperture);
    perception_publisher.publish(perception_msg);
    initializePerceptionMsg();
  }
}

void stopReceivedCallback(const std_msgs::Bool& msg)
{
  stop = msg.data;
}

void depthReceivedCallback(const sensor_msgs::Image& msg)
{
  perception_msg.depth = msg;
  if (perception_msg.rgb.height != 0)
  {
    perception_msg.drone_state.drone_pose = drone_pose;
    perception_msg.drone_state.intrinsics = getInstrinscsMsg(focal_length, focus_distance, aperture);
    perception_publisher.publish(perception_msg);
    initializePerceptionMsg();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_main");

  initializeTargets();
  initializeKalmanFilter();

  auto const now = std::chrono::system_clock::now();
  auto const in_time_t = std::chrono::system_clock::to_time_t(now);
  logErrorFileName << "/home/pablo/Desktop/AirSim_update/AirSim_ros/ros/src/cinempc/logs/error_pos_"
                   << std::put_time(std::localtime(&in_time_t), "%d_%m_%Y-%H_%M_%S") << ".csv";
  errorFile.open(logErrorFileName.str());  // pitch,roll,yaw for every time stamp
  errorFile << "Time"
            << ","
            << "gt_x"
            << ","
            << "gt_y"
            << ","
            << "gt_z"
            << ","
            << "perception_x"
            << ","
            << "perception_y"
            << ","
            << "perception_z"
            << ","
            << "error_x"
            << ","
            << "error_y"
            << ","
            << "error_z" << std::endl;

  ros::NodeHandle n;

  start_log = ros::Time::now();
  ros::ServiceClient service_take_off = n.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
  airsim_ros_pkgs::Takeoff srv;
  srv.request.waitOnLastTask = false;

  // service_take_off.call(srv);

  ros::Subscriber change_sequence_sub = n.subscribe("cinempc/sequence", 1000, changeSeqCallback);

  ros::Subscriber new_drone_state_received_sub = n.subscribe("airsim_node/drone_1/"
                                                             "odom_local_ned",
                                                             1000, readDroneStateCallback);

  ros::Subscriber mpc_result_n_steps_sub = n.subscribe("cinempc/next_n_states", 1000, mpcResultCallback);

  ros::Subscriber new_rgb_received = n.subscribe("/airsim_node/drone_1/Scene", 1000, rgbReceivedCallback);

  ros::Subscriber stop_signal_received = n.subscribe("/cinempc/stop", 1000, stopReceivedCallback);

  ros::Subscriber new_depth_received = n.subscribe("/airsim_node/drone_1/DepthVis", 1000, depthReceivedCallback);

  std::vector<ros::Subscriber> targets_states_subscribers = {};
  for (int i = 0; i < targets_names.size(); i++)
  {
    targets_states_subscribers.push_back(n.subscribe<geometry_msgs::PoseStamped>(
        "airsim_node/" + targets_names.at(i) + "/get_pose", 1000, boost::bind(&readTargetStateCallback, _1, i)));

    targets_states_subscribers.push_back(
        n.subscribe<cinempc::PersonStatePerception>("/cinempc/" + targets_names.at(i) + "/target_state_perception",
                                                    1000, boost::bind(&readTargetStatePerceptionCallback, _1, i)));
  }

  perception_publisher = n.advertise<cinempc::PerceptionMsg>("/cinempc/perception_in", 10);

  fpv_intrinsics_publisher = n.advertise<airsim_ros_pkgs::IntrinsicsCamera>("/airsim_node/drone_1/set_intrinsics", 10);
  fpv_intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance, aperture));

  gimbal_rotation_publisher =
      n.advertise<airsim_ros_pkgs::GimbalAngleQuatCmd>("/airsim_node/gimbal_angle_quat_cmd", 10);

  new_MPC_state_publisher = n.advertise<cinempc::MPCIncomingState>("cinempc/current_state", 10);

  cinempc_calculate_new_states_timer_ = n.createTimer(ros::Duration(dt), boost::bind(publishNewStateToMPC, _1, n));

  service_move_on_path = n.serviceClient<airsim_ros_pkgs::MoveOnPath>("/airsim_node/drone_1/move_on_path");

  // init camera pose
  airsim_ros_pkgs::GimbalAngleQuatCmd msg;
  geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, 0, drone_start_yaw);
  msg.orientation = q;
  drone_pose.orientation = q;
  gimbal_rotation_publisher.publish(msg);

  ros::Rate loop_rate(freq_loop);
  while (ros::ok() && !stop)
  {
    if (focal_length_spline.get_x().size() != 0)
    {
      focal_length = focal_length_spline(interval * index_splines);
      focus_distance = focus_distance_spline(interval * index_splines);
      aperture = aperture_spline(interval * index_splines);

      fpv_intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance * 100, aperture));

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
  errorFile.close();
  return 0;
}
