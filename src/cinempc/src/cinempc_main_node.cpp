#include "cinempc_main_node.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

// Current position of drone
std::vector<cinempc::TargetState> world_targets_states_perception, world_targets_states_gt;

std::vector<KalmanFilterEigen> kalman_filter_targets;

cinempc::PlotValues plot_values;

float vel_x, vel_y, vel_z;
float focal_length = 35, focus_distance = 10000, aperture = 22;

int index_splines = 0;
bool noise = true;
float sequence = 1;
int change_sequence_index = 0;
double steps_each_dt = 40;
double interval = mpc_dt / steps_each_dt;
double freq_loop = 1 / interval;
bool stop = false, low_cost = false;

double yaw_gimbal = drone_start_yaw, pitch_gimbal = 0;

ros::Time start_log;
ros::Time start_measure;

std::stringstream logErrorFileName;
std::ofstream logFile;

cinempc::PerceptionMsg perception_msg;

float focal_length_next_state = 35;
geometry_msgs::Pose drone_pose_next_state;
geometry_msgs::Pose drone_pose_when_called;
;

std::vector<double> times_vector, focal_length_vector, focus_distance_vector, aperture_vector, roll_vector, yaw_vector,
    pitch_vector;
tk::spline focal_length_spline, focus_distance_spline, aperture_spline, roll_spline, yaw_spline, pitch_spline;

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
  std::cout << "Sequence:" << sequence << std::endl;
  plot_values.sequence = sequence;
}

void logRPY(Eigen::Matrix<double, 3, 3> R, string name)
{
  cinempc::RPY<double> wRs2 = cinempc::RMatrixtoRPY<double>(R);

  std::cout << name << std::endl
            << "--------- " << std::endl
            << "   R-P-Y:wRd  " << wRs2.roll << " pitch:   " << wRs2.pitch << " YAW: " << wRs2.yaw << std::endl;
}

void logPosition(geometry_msgs::Point pos, string name)
{
  double distance_2D_target = cinempc::calculateDistanceTo2DPoint<double>(0, 0, pos.x, pos.y);
  std::cout << name << std::endl
            << "--------- " << std::endl
            << "   x  " << pos.x << " y:   " << pos.y << " z: " << pos.z << "  Distance: " << distance_2D_target
            << std::endl;
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
KalmanFilterEigen initializeKalmanFilterTarget()
{
  int n = 6;  // Number of states
  int m = 3;  // Number of measurements

  double dt_kf = 0.2;  // Time step that we receive each image/measurement. We need to make it match with
                       // update_airsim_img_response_every_n_sec param of the sim

  Eigen::MatrixXd A(n, n);  // System dynamics matrix
  Eigen::MatrixXd C(m, n);  // Output matrix
  Eigen::MatrixXd Q(n, n);  // Process noise covariance
  Eigen::MatrixXd R(m, m);  // Measurement noise covariance
  Eigen::MatrixXd P(n, n);  // Estimate error covariance

  A << 1, 0, 0, dt_kf, 0, 0, 0, 1, 0, 0, dt_kf, 0, 0, 0, 1, 0, 0, dt_kf, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 1;
  C << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  // Reasonable covariance matrices
  Q << 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0,
      0, 0.5;  // hacer dependiente de dt
  R << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;

  P << 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0,
      2;  // bigger than  Q and R. Incertidumbre inicial

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilterEigen init(dt_kf, A, C, Q, R, P, n, m);

  Eigen::VectorXd x0(n);
  double t = 0;
  x0 << 0, 0, 0, 0, 0, 0;
  init.init(t, x0);

  return init;
}

const Eigen::MatrixXd getNewAMatrix(double dt_kf)
{
  int n = 6;                // Number of states
  Eigen::MatrixXd A(n, n);  // System dynamics matrix

  A << 1, 0, 0, dt_kf, 0, 0, 0, 1, 0, 0, dt_kf, 0, 0, 0, 1, 0, 0, dt_kf, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 1;

  return A;
}

std::vector<geometry_msgs::Point> updateKalmanWithNewMeasureAndGetState(geometry_msgs::Pose world_pose_target,
                                                                        int target, double t_s)
{
  Eigen::VectorXd measurement(kalman_filter_targets.at(target).numberOfMeasurements());
  measurement(0) = world_pose_target.position.x;
  measurement(1) = world_pose_target.position.y;
  measurement(2) = world_pose_target.position.z;

  Eigen::MatrixXd A = getNewAMatrix(t_s);
  kalman_filter_targets.at(target).update(measurement, t_s, A);

  Eigen::VectorXd new_state(kalman_filter_targets.at(target).numberOfStates());

  new_state = kalman_filter_targets.at(target).state();

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

void updateKalmanWithoutMeasure(int target, double t_s)
{
  Eigen::MatrixXd A = getNewAMatrix(t_s);
  kalman_filter_targets.at(target).update(t_s, A);
}

geometry_msgs::Quaternion predictWorldOrientationFromVelocity(double vx, double vy, double vz)
{
  Eigen::Matrix<double, 3, 1> wvt(vx, vy, vz);
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

  // logRPY(R, "orient");
  return cinempc::RMatrixToQuat<double>(R);
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

std::vector<geometry_msgs::Pose> predictWorldTopPosesFromKF(int target_index, int kf_time_each_mpc)
{
  Eigen::MatrixXd new_states(MPC_N, kalman_filter_targets.at(target_index).numberOfStates());

  new_states = kalman_filter_targets.at(target_index).predict(kf_time_each_mpc, MPC_N);

  geometry_msgs::Point new_position, new_vel;

  std::vector<geometry_msgs::Pose> poses;

  for (int i = 0; i < MPC_N; i++)
  {
    geometry_msgs::Point new_vel;
    geometry_msgs::Pose target_pose;

    target_pose.position.x = new_states(i, 0);
    target_pose.position.y = new_states(i, 1);
    target_pose.position.z = new_states(i, 2);
    new_vel.x = new_states(i, 3);
    new_vel.y = new_states(i, 4);
    new_vel.z = new_states(i, 5);

    if (!static_target)
    {
      target_pose.orientation = predictWorldOrientationFromVelocity(new_vel.x, new_vel.y, new_vel.z);
    }
    else
    {
      target_pose.orientation = cinempc::RPYToQuat<double>(0, 0, subject_yaw_gt);
    }

    poses.push_back(target_pose);

    // logPosition(target_pose.position, to_string(i));

    // logPosition(new_vel, "vel " + to_string(i));
  }

  // logPosition(new_position, "pos_kf" + to_string(time_step));
  // if (time_step == 0)
  // {
  //   logPosition(new_vel, "vel_kf" + to_string(time_step));
  // }

  return poses;
}

cinempc::TargetState fulfillRelativePosesFromWorldTop(geometry_msgs::Pose world_pose_top,
                                                      geometry_msgs::Pose drone_pose_now)
{
  cinempc::TargetState result;

  geometry_msgs::Pose world_pose_center, world_pose_bottom;
  geometry_msgs::Pose drone_pose_top, drone_pose_center, drone_pose_bottom;

  float target_z_center = world_pose_top.position.z + 0.5;
  float target_z_bottom = world_pose_top.position.z + target_height;

  world_pose_center.position.x = world_pose_top.position.x;
  world_pose_center.position.y = world_pose_top.position.y;
  world_pose_center.position.z = target_z_center;
  world_pose_center.orientation = world_pose_top.orientation;

  world_pose_bottom.position.x = world_pose_top.position.x;
  world_pose_bottom.position.y = world_pose_top.position.y;
  world_pose_bottom.position.z = target_z_bottom;
  world_pose_bottom.orientation = world_pose_top.orientation;

  result.pose_top = cinempc::calculate_relative_pose_drone_person<double>(world_pose_top, drone_pose_now);
  result.pose_center = cinempc::calculate_relative_pose_drone_person<double>(world_pose_center, drone_pose_now);
  result.pose_bottom = cinempc::calculate_relative_pose_drone_person<double>(world_pose_bottom, drone_pose_now);

  return result;
}

void readTargetStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int target_index)
{
  geometry_msgs::Pose wTtop_measure;

  float target_x_bottom_gt = msg->pose.position.x;
  float target_y_bottom_gt = msg->pose.position.y;
  float target_z_bottom_gt = msg->pose.position.z;

  float target_z_top_gt = target_z_bottom_gt - target_height;

  // world
  wTtop_measure.position.x = target_x_bottom_gt;
  wTtop_measure.position.y = target_y_bottom_gt + target_width / 2;
  wTtop_measure.position.z = target_z_top_gt;
  wTtop_measure.orientation = cinempc::RPYToQuat<double>(0, 0, subject_yaw_gt);

  plot_values.target_world_gt = wTtop_measure.position;

  // plot
  geometry_msgs::Pose relative_pose = cinempc::calculate_relative_pose_drone_person<double>(wTtop_measure, drone_pose);
  plot_values.d_gt =
      cinempc::calculateDistanceTo2DPoint<double>(0, 0, relative_pose.position.x, relative_pose.position.y);

  if (!use_perception)
  {
    ros::Time end_measure = ros::Time::now();
    ros::Duration time_measure = end_measure - start_measure;
    double time_s = time_measure.toSec();
    updateKalmanWithNewMeasureAndGetState(wTtop_measure, target_index, time_s);
    start_measure = ros::Time::now();
  }
}

void readTargetStatePerceptionCallback(const cinempc::PerceptionOut::ConstPtr& msg)
{
  // std::cout << "pose:" << msg->pose_top.position.x << std::endl;
  // world position target

  std::vector<geometry_msgs::Point> state;
  ros::Time end_measure = ros::Time::now();
  ros::Duration time_measure = end_measure - start_measure;
  double time_s = time_measure.toSec();

  if (msg->targets_found > 0)
  {
    for (cinempc::TargetState target_state : msg->targets_state)
    {
      if (target_state.target_name.find("person") != std::string::npos)
      {
        geometry_msgs::Pose wTtop_measure = cinempc::calculate_world_pose_from_relative<double>(
            msg->drone_state.drone_pose, target_state.pose_top, false);
        wTtop_measure.position.z += 0.2;  // face
        state = updateKalmanWithNewMeasureAndGetState(wTtop_measure, 0, time_s);
        plot_values.target_world_perception = wTtop_measure.position;
        // logPosition(wTtop_measure.position, "wTtop_measure");
        plot_values.v_kf = state.at(1);
      }
    }
  }

  else
  {
    updateKalmanWithoutMeasure(0, time_s);
  }

  start_measure = ros::Time::now();
}
void initializeTargets()
{
  for (int i = 0; i < targets_names.size(); i++)
  {
    cinempc::TargetState state;
    world_targets_states_perception.push_back(state);
    world_targets_states_gt.push_back(state);

    KalmanFilterEigen kf = initializeKalmanFilterTarget();
    kalman_filter_targets.push_back(kf);
  }
}

void mpcResultCallback(const cinempc::MPCResult::ConstPtr& msg)
{
  std::cout << std::endl;
  std::vector<geometry_msgs::Point> pathMPC;
  focal_length_vector.clear();
  focus_distance_vector.clear();
  aperture_vector.clear();
  yaw_vector.clear();
  roll_vector.clear();
  pitch_vector.clear();
  times_vector.clear();

  int index_mpc = 0;

  // plot values
  ros::Time end_log = ros::Time::now();
  ros::Duration diff = end_log - start_log;
  plot_values.time_ms = diff.toNSec() / 1000000;
  plot_values.mpc_plot_values = msg->plot_values;

  // std::cout << msg->cost << std::endl;
  if (msg->cost > 0)
  {
    logFile << cinempc::plotValues(plot_values, false);
  }
  if (msg->cost >= 100 || msg->cost == 0)
  {
    low_cost = false;
    roll_vector.insert(roll_vector.begin(), cinempc::quatToRPY<double>(drone_pose_when_called.orientation).roll);
    pitch_vector.insert(pitch_vector.begin(), cinempc::quatToRPY<double>(drone_pose_when_called.orientation).pitch);
    yaw_vector.insert(yaw_vector.begin(), cinempc::quatToRPY<double>(drone_pose_when_called.orientation).yaw);
    focal_length_vector.insert(focal_length_vector.begin(), focal_length);
    focus_distance_vector.insert(focus_distance_vector.begin(), focus_distance);
    aperture_vector.insert(aperture_vector.begin(), aperture);
    times_vector.push_back(0);

    index_mpc++;

    double max_vel_x = 0, max_vel_y = 0, max_vel_z = 0;

    while (index_mpc < MPC_N)
    {
      cinempc::DroneAndCameraState cine_mpc_result = msg->mpc_n_states.at(index_mpc);
      times_vector.push_back(mpc_dt * index_mpc);

      focal_length_vector.insert(focal_length_vector.begin() + index_mpc, cine_mpc_result.intrinsics.focal_length);

      focus_distance_vector.insert(focus_distance_vector.begin() + index_mpc,
                                   cine_mpc_result.intrinsics.focus_distance);
      aperture_vector.insert(aperture_vector.begin() + index_mpc, cine_mpc_result.intrinsics.aperture);

      geometry_msgs::Pose world_T_result;

      // std::cout << "result:" << cine_mpc_result.drone_pose.position.x << "    " <<
      // cine_mpc_result.drone_pose.position.y
      //           << "    " << cine_mpc_result.drone_pose.position.z << std::endl;

      world_T_result =
          cinempc::calculate_world_pose_from_relative<double>(drone_pose_when_called, cine_mpc_result.drone_pose);

      cinempc::RPY<double> rpy = cinempc::quatToRPY<double>(world_T_result.orientation);

      roll_vector.insert(roll_vector.begin() + index_mpc, rpy.roll);
      yaw_vector.insert(yaw_vector.begin() + index_mpc, rpy.yaw);
      pitch_vector.insert(pitch_vector.begin() + index_mpc, rpy.pitch);

      geometry_msgs::Point path_point(world_T_result.position);
      pathMPC.push_back(path_point);

      // logPosition(world_T_result.position, to_string(index_mpc));

      if (index_mpc == 2)
      {
        focal_length_next_state = cine_mpc_result.intrinsics.focal_length;
        drone_pose_next_state = world_T_result;
        vel_x = cine_mpc_result.velocity.x;
        vel_y = cine_mpc_result.velocity.y;
        vel_z = cine_mpc_result.velocity.z;
      }

      geometry_msgs::Pose dTvel;
      dTvel.position = cine_mpc_result.velocity;
      dTvel.orientation = world_T_result.orientation;
      geometry_msgs::Pose wTvel =
          cinempc::calculate_world_pose_from_relative<double>(drone_pose_when_called, dTvel, true);

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

    for (double focal_l : pitch_vector)
    {
      // std::cout << "pitch:" << focal_l << std::endl;
    }
    for (double focal_l : roll_vector)
    {
      // std::cout << "roll:" << focal_l << std::endl;
    }
    focal_length_spline.set_points(times_vector, focal_length_vector);
    focus_distance_spline.set_points(times_vector, focus_distance_vector);
    aperture_spline.set_points(times_vector, aperture_vector);
    roll_spline.set_points(times_vector, roll_vector);
    yaw_spline.set_points(times_vector, yaw_vector);
    pitch_spline.set_points(times_vector, pitch_vector);

    index_splines = 0;

    // move on path
    airsim_ros_pkgs::MoveOnPath srv;
    double max_vel = max(0.1, max(max_vel_x, max(max_vel_y, max_vel_z)));

    // std::cout << "max_vel:" << max_vel << std::endl;
    srv.request.vel = max_vel;
    srv.request.timeout = 10;
    srv.request.rads_yaw = cinempc::quatToRPY<double>(drone_pose.orientation).yaw;

    double distance = cinempc::calculateDistanceTo3DPoint<double>(pathMPC.at(0).x, pathMPC.at(0).y, pathMPC.at(0).z,
                                                                  pathMPC.at(MPC_N - 2).x, pathMPC.at(MPC_N - 2).y,
                                                                  pathMPC.at(MPC_N - 2).z);

    //  std::cout << "distance:" << distance << std::endl;
    if (distance > 0.20)
    {
      srv.request.positions = pathMPC;
      service_move_on_path.call(srv);
    }
  }
  else
  {
    low_cost = true;
  }
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
  geometry_msgs::Pose drone_pose_now = drone_pose;
  drone_pose_when_called = drone_pose_now;

  // logPosition(drone_pose_now.position, "Drone_pose_now");
  cinempc::MPCIncomingState msg_mpc_in;
  msg_mpc_in.drone_state.drone_pose.position.x = 0;  // drone_pose.position.x;
  msg_mpc_in.drone_state.drone_pose.position.y = 0;  // drone_pose.position.y;
  msg_mpc_in.drone_state.drone_pose.position.z = 0;  // drone_pose.position.z;

  msg_mpc_in.drone_state.velocity.x = vel_x;  // drone_pose.position.x;
  msg_mpc_in.drone_state.velocity.y = vel_y;  // drone_pose.position.x;
  msg_mpc_in.drone_state.velocity.z = vel_z;  // drone_pose.position.x;

  geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, 0, 0);
  msg_mpc_in.drone_state.drone_pose.orientation = q;  // drone_pose.orientation;
  msg_mpc_in.floor_pos = -drone_pose.position.z;

  // std::cout << "floor_pos:" << msg_mpc_in.floor_pos << std::endl;

  msg_mpc_in.drone_state.intrinsics = getInstrinscsMsg(focal_length_next_state, focus_distance, aperture);

  // initalize mesage
  for (int target = 0; target < targets_names.size(); target++)
  {
    cinempc::TargetStateMPC target_state_mpc;
    for (int j = 0; j < MPC_N; j++)
    {
      geometry_msgs::Pose pose;
      target_state_mpc.poses_top.push_back(pose);
      target_state_mpc.poses_center.push_back(pose);
      target_state_mpc.poses_bottom.push_back(pose);
    }
    msg_mpc_in.targets.push_back(target_state_mpc);
  }
  std::vector<geometry_msgs::Quaternion> world_rotations;

  for (int target = 0; target < targets_names.size(); target++)
  {
    KalmanFilterEigen kf_target = kalman_filter_targets.at(target);
    int kf_time_each_mpc = mpc_dt / kf_target.get_dt();
    std::vector<geometry_msgs::Pose> world_top_poses = predictWorldTopPosesFromKF(target, kf_time_each_mpc);
    world_rotations.push_back(world_top_poses.at(0).orientation);
    for (int t = 0; t < MPC_N; t++)
    {
      geometry_msgs::Pose world_pose_top_kf = world_top_poses.at(t);
      // logPosition(world_pose_top_kf.position, to_string(t));
      if (t == 0)
      {
        plot_values.target_world_kf = world_pose_top_kf.position;
      }

      plot_values.target_rot_gt = cinempc::RPYToQuat<double>(0, 0, subject_yaw_gt);
      plot_values.target_rot_perception = world_pose_top_kf.orientation;

      cinempc::TargetState target_state = fulfillRelativePosesFromWorldTop(world_pose_top_kf, drone_pose_now);

      msg_mpc_in.targets.at(target).poses_top.at(t) = target_state.pose_top;
      msg_mpc_in.targets.at(target).poses_center.at(t) = target_state.pose_center;
      msg_mpc_in.targets.at(target).poses_bottom.at(t) = target_state.pose_bottom;

      // logPosition(msg_mpc_in.targets.at(target).poses_top.at(t).position, "pos" + to_string(t));
    }
  }
  ros::ServiceClient service_get_user_constraints = n.serviceClient<cinempc::GetUserConstraints>("/cinempc/"
                                                                                                 "user_node/"
                                                                                                 "get_"
                                                                                                 "constraints");
  cinempc::GetUserConstraints srv;
  srv.request.targets_relative = msg_mpc_in.targets;
  srv.request.world_rotations_target = world_rotations;
  srv.request.drone_pose = drone_pose_now;
  srv.request.sequence = sequence;
  if (service_get_user_constraints.call(srv))
  {
    msg_mpc_in.constraints = srv.response.contraints;
    new_MPC_state_publisher.publish(msg_mpc_in);
    plot_values.constraints = srv.response.contraints;
    // std::cout << "PUBLISHED" << std::endl;
  }
}

void rgbReceivedCallback(const sensor_msgs::Image& msg)
{
  if (use_perception)
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
}

void stopReceivedCallback(const std_msgs::Bool& msg)
{
  stop = msg.data;
}

void depthReceivedCallback(const sensor_msgs::Image& msg)
{
  if (use_perception)
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
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_main");

  initializeTargets();

  auto const now = std::chrono::system_clock::now();
  auto const in_time_t = std::chrono::system_clock::to_time_t(now);
  logErrorFileName << "/home/pablo/Desktop/AirSim_update/AirSim_ros/ros/src/cinempc/logs/error_pos_"
                   << std::put_time(std::localtime(&in_time_t), "%d_%m_%Y-%H_%M_%S") << ".csv";
  logFile.open(logErrorFileName.str());  // pitch,roll,yaw for every time stamp

  logFile << cinempc::plotValues(plot_values, true);

  ros::NodeHandle n;

  start_log = ros::Time::now();
  start_measure = ros::Time::now();
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

  ros::Subscriber new_depth_received = n.subscribe("/airsim_node/drone_1/DepthVis", 1000, depthReceivedCallback);

  ros::Subscriber stop_signal_received = n.subscribe("/cinempc/stop", 1000, stopReceivedCallback);

  std::vector<ros::Subscriber> targets_states_subscribers_gt = {};
  ros::Subscriber targets_states_subscriber_perception;
  if (use_perception)
  {
    targets_states_subscriber_perception = n.subscribe<cinempc::PerceptionOut>(
        "/cinempc/perception_output", 1000, boost::bind(&readTargetStatePerceptionCallback, _1));
  }
  for (int i = 0; i < targets_names.size(); i++)
  {
    targets_states_subscribers_gt.push_back(n.subscribe<geometry_msgs::PoseStamped>(
        "airsim_node/" + targets_names.at(i) + "/get_pose", 1000, boost::bind(&readTargetStateCallback, _1, i)));
  }

  perception_publisher = n.advertise<cinempc::PerceptionMsg>("/cinempc/perception_in", 10);

  fpv_intrinsics_publisher = n.advertise<airsim_ros_pkgs::IntrinsicsCamera>("/airsim_node/drone_1/set_intrinsics", 10);
  fpv_intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance, aperture));

  gimbal_rotation_publisher =
      n.advertise<airsim_ros_pkgs::GimbalAngleQuatCmd>("/airsim_node/gimbal_angle_quat_cmd", 10);

  new_MPC_state_publisher = n.advertise<cinempc::MPCIncomingState>("cinempc/current_state", 10);

  cinempc_calculate_new_states_timer_ = n.createTimer(ros::Duration(mpc_dt), boost::bind(publishNewStateToMPC, _1, n));

  service_move_on_path = n.serviceClient<airsim_ros_pkgs::MoveOnPath>("/airsim_node/drone_1/move_on_path");

  // init camera pose
  airsim_ros_pkgs::GimbalAngleQuatCmd msg;
  geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, 0, drone_start_yaw);
  msg.orientation = q;
  drone_pose.orientation = q;
  drone_pose_next_state.orientation = q;
  drone_pose_next_state = drone_pose;
  gimbal_rotation_publisher.publish(msg);

  double yaw_step = 0, pitch_step = 0, focal_step = 0, ap_step;

  ros::Rate loop_rate(freq_loop);
  while (ros::ok() && !stop)
  {
    if (focal_length_spline.get_x().size() != 0 && low_cost == false)
    {
      double diff_yaw, diff_pitch, diff_focal, diff_ap;
      if (index_splines == 0)
      {
        diff_yaw = yaw_vector.at(1) - yaw_vector.at(0);
        diff_pitch = pitch_vector.at(1) - pitch_vector.at(0);
        diff_focal = focal_length_vector.at(1) - focal_length_vector.at(0);
        diff_focal = focal_length_vector.at(1) - focal_length_vector.at(0);
        diff_ap = aperture_vector.at(1) - aperture_vector.at(0);

        yaw_step = diff_yaw / steps_each_dt;
        pitch_step = diff_pitch / steps_each_dt;
        focal_step = diff_focal / steps_each_dt;
        ap_step = diff_ap / steps_each_dt;

        if (abs(diff_yaw) < 0.001)
        {
          yaw_step = 0;
        }
        if (abs(diff_pitch) < 0.001)
        {
          pitch_step = 0;
        }
        if (abs(diff_focal) < 0.5)
        {
          focal_step = 0;
        }
      }
      focal_length = focal_length + focal_step;
      focus_distance = focus_distance_spline(interval * index_splines);
      aperture = aperture + ap_step;

      // focal_length = focal_length + focal_step;
      // focus_distance = focus_distance_spline(interval * index_splines);
      // aperture = aperture_spline(interval * index_splines);

      fpv_intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance * 100, aperture));

      // plot_values.intrinsics_camera = getInstrinscsMsg(focal_length, focus_distance * 100, aperture);

      // double roll_gimbal = roll_spline(interval * index_splines);

      // double roll_gimbal = roll_spline(interval * index_splines);

      // double yaw_gimbal = yaw_spline(interval * index_splines);
      // double pitch_gimbal = pitch_spline(interval * index_splines);

      // if (index_splines == 0)
      // {
      //   double diff_pitch = pitch_vector.at(1) - pitch_vector.at(0);
      //   if (abs(diff_pitch) < 0.001)
      //   {
      //     pitch_gimbal = pitch_vector.at(0);
      //   }
      //   double diff_yaw = yaw_vector.at(1) - yaw_vector.at(0);
      //   if (abs(diff_yaw) < 0.001)
      //   {
      //     yaw_gimbal = yaw_vector.at(0);
      //   }
      // }

      // double yaw_gimbal = yaw_spline(interval * index_splines);

      // double yaw_gimbal_result = yaw_gimbal + yaw_step;
      // double pitch_gimbal_result = pitch_gimbal + pitch_step;

      yaw_gimbal = yaw_gimbal + yaw_step;
      pitch_gimbal = pitch_gimbal + pitch_step;

      // round rotation to 4 decimals
      // int yaw_gimbal_int = round(yaw_gimbal * 10000);
      // int pitch_gimbal_int = round(pitch_gimbal * 10000);

      // yaw_gimbal = yaw_gimbal_int / 10000.0;
      // pitch_gimbal = pitch_gimbal_int / 10000.0;

      geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, pitch_gimbal, yaw_gimbal);
      // std::cout << "yaw1:" << cinempc::quatToRPY<double>(drone_pose.orientation).yaw << std::endl;
      // std::cout << "pitch:" << pitch_gimbal << std::endl;
      // std::cout << "yaw:" << yaw_gimbal << std::endl;

      airsim_ros_pkgs::GimbalAngleQuatCmd msg;
      msg.orientation = q;
      drone_pose.orientation = q;

      gimbal_rotation_publisher.publish(msg);
    }

    index_splines++;
    loop_rate.sleep();
    ros::spinOnce();
  }
  logFile.close();
  int a = system("rm /home/pablo/Documents/journal/matlab/log_file.csv");
  string str1 = "cp " + logErrorFileName.str() + "  /home/pablo/Documents/journal/matlab/log_file.csv";
  a = system(str1.c_str());
  return 0;
}