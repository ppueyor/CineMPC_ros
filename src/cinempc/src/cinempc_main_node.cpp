#include "cinempc_main_node.h"

using namespace std;
using namespace std::chrono;

// Current position of drone
std::vector<cinempc::TargetState> world_targets_states_perception, world_targets_states_gt;

cinempc::PlotValues plot_values;

float vel_x, vel_y, vel_z;
float focal_length = 35, focus_distance = 10000, aperture = 22;

bool noise = true;
float sequence = 1;
int change_sequence_index = 0;
bool stop = false, low_cost = false;

double yaw_gimbal = drone_start_yaw, pitch_gimbal = 0;

ros::Time start_log;
ros::Time start_measure;

std::stringstream logErrorFileName;
std::ofstream logFile;

cinempc::MeasurementIn perception_meas_in_msg;

airsim_ros_pkgs::IntrinsicsCamera intrinsics_next_state;
float focal_length_next_state = 35;
geometry_msgs::Pose drone_pose_next_state;
geometry_msgs::Pose drone_pose_when_called;

std::vector<float> times_vector, roll_vector, yaw_vector, pitch_vector, focal_length_vector, focus_distance_vector,
    aperture_vector;

static std::default_random_engine generator;
void myPoseMsgToTF(const geometry_msgs::Pose& msg, tf2::Transform& bt)
{
  bt = tf2::Transform(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                      tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
}

void initializePerceptionMsg()
{
  perception_meas_in_msg.depth = sensor_msgs::Image();
  perception_meas_in_msg.rgb = sensor_msgs::Image();
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

void readDroneStateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  float drone_x = msg->pose.pose.position.x;
  float drone_y = msg->pose.pose.position.y;
  float drone_z = msg->pose.pose.position.z;

  drone_pose.position.x = drone_x;
  drone_pose.position.y = drone_y;
  drone_pose.position.z = drone_z;

  plot_values.drone_position_gt = msg->pose.pose.position;

  // drone_pose.orientation = msg->pose.pose.orientation;
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
    double time_s = time_measure.toSec() / sim_speed;
    cinempc::EstimationIn estimation_in_msg;
    estimation_in_msg.world_pose_target = wTtop_measure;
    estimation_in_msg.target_index = target_index;
    estimation_in_msg.time_s = time_s;
    estimation_in_msg.measure = true;
    estimation_in_publisher.publish(estimation_in_msg);
    start_measure = ros::Time::now();
  }
}

void readTargetStatePerceptionCallback(const cinempc::MeasurementOut::ConstPtr& msg)
{
  // std::cout << "pose:" << msg->pose_top.position.x << std::endl;
  // world position target

  std::vector<geometry_msgs::Point> state;
  ros::Time end_measure = ros::Time::now();
  ros::Duration time_measure = end_measure - start_measure;
  double time_s = time_measure.toSec() / sim_speed;

  if (msg->targets_found > 0)
  {
    for (cinempc::TargetState target_state : msg->targets_state)
    {
      if (target_state.target_name.find("person") != std::string::npos)
      {
        geometry_msgs::Pose wTtop_measure = cinempc::calculate_world_pose_from_relative<double>(
            msg->drone_state.drone_pose, target_state.pose_top, false);
        wTtop_measure.position.z += 0.2;  // face

        cinempc::EstimationIn estimation_in_msg;
        estimation_in_msg.world_pose_target = wTtop_measure;
        estimation_in_msg.target_index = 0;
        estimation_in_msg.time_s = time_s;
        estimation_in_msg.measure = true;
        estimation_in_publisher.publish(estimation_in_msg);

        // TODO
        // state = updateKalmanWithNewMeasureAndGetState(wTtop_measure, 0, time_s);
        // plot_values.target_world_perception = wTtop_measure.position;
        // logPosition(wTtop_measure.position, "wTtop_measure");
        // plot_values.v_kf = state.at(1);
      }
    }
  }
  else
  {
    // update the times of KF even if there is no measure
    for (int i = 0; i < targets_names.size(); i++)
    {
      cinempc::EstimationIn estimation_in_msg;
      estimation_in_msg.time_s = time_s;
      estimation_in_msg.target_index = i;
      estimation_in_msg.measure = false;
      estimation_in_publisher.publish(estimation_in_msg);
    }
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
  plot_values.time_ms = diff.toNSec() / (1000000 * sim_speed);
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

      if (index_mpc == 1)
      {
        intrinsics_next_state = cine_mpc_result.intrinsics;
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

    cinempc::LowLevelControl low_level_control_msg;

    low_level_control_msg.yaw_vector = yaw_vector;
    low_level_control_msg.pitch_vector = pitch_vector;
    low_level_control_msg.focal_length_vector = focal_length_vector;
    low_level_control_msg.aperture_vector = aperture_vector;
    low_level_control_msg.focus_distance_vector = focus_distance_vector;
    low_level_control_msg.times_vector = times_vector;

    // move on path
    airsim_ros_pkgs::MoveOnPath msg_move_on_path;
    double max_vel = max(0.1, max(max_vel_x, max(max_vel_y, max_vel_z)));

    // std::cout << "max_vel:" << max_vel << std::endl;
    low_level_control_msg.move_on_path_msg.vel = max_vel;
    low_level_control_msg.move_on_path_msg.timeout = 10;
    low_level_control_msg.move_on_path_msg.rads_yaw = cinempc::quatToRPY<double>(drone_pose.orientation).yaw;
    low_level_control_msg.move_on_path_msg.positions = pathMPC;

    low_level_control_publisher.publish(low_level_control_msg);
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

  msg_mpc_in.drone_state.intrinsics = intrinsics_next_state;

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
    ros::ServiceClient service_get_N_poses = n.serviceClient<cinempc::GetNNextTargetPoses>("/cinempc/"
                                                                                           "get_N_target_poses");

    cinempc::GetNNextTargetPoses srv_get_poses;
    srv_get_poses.request.target_index = target;
    srv_get_poses.request.mpc_dt = mpc_dt;
    srv_get_poses.request.mpc_N = MPC_N;
    if (service_get_N_poses.call(srv_get_poses))
    {
      std::vector<geometry_msgs::Pose> world_top_poses = srv_get_poses.response.poses_target;
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
    perception_meas_in_msg.rgb = msg;
    if (perception_meas_in_msg.depth.height != 0)
    {
      perception_meas_in_msg.drone_state.drone_pose = drone_pose;
      perception_meas_in_msg.drone_state.intrinsics = getInstrinscsMsg(focal_length, focus_distance, aperture);
      perception_meas_publisher.publish(perception_meas_in_msg);
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
    perception_meas_in_msg.depth = msg;
    if (perception_meas_in_msg.rgb.height != 0)
    {
      perception_meas_in_msg.drone_state.drone_pose = drone_pose;
      perception_meas_in_msg.drone_state.intrinsics = getInstrinscsMsg(focal_length, focus_distance, aperture);
      perception_meas_publisher.publish(perception_meas_in_msg);
      initializePerceptionMsg();
    }
  }
}

void readGimbalOrientationCallback(const airsim_ros_pkgs::GimbalAngleQuatCmd::ConstPtr& msg)
{
  drone_pose.orientation = msg->orientation;
}

void readIntrinsicsCallback(const airsim_ros_pkgs::IntrinsicsCamera::ConstPtr& msg)
{
  focal_length = msg->focal_length;
  aperture = msg->aperture;
  focus_distance = msg->focus_distance / 100;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_main");

  initializeTargets();

  auto const now = std::chrono::system_clock::now();
  auto const in_time_t = std::chrono::system_clock::to_time_t(now);
  logErrorFileName << "/home/pablo/Desktop/AirSim_update/AirSim_ros/ros/src/cinempc/logs/Tuareg/error_pos_"
                   << std::put_time(std::localtime(&in_time_t), "%d_%m_%Y-%H_%M_%S") << ".csv";
  logFile.open(logErrorFileName.str());  // pitch,roll,yaw for every time stamp

  logFile << cinempc::plotValues(plot_values, true);

  ros::NodeHandle n;

  start_log = ros::Time::now();
  start_measure = ros::Time::now();
  ros::ServiceClient service_take_off = n.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
  airsim_ros_pkgs::Takeoff srv;
  srv.request.waitOnLastTask = false;

  intrinsics_next_state.aperture = 22;
  intrinsics_next_state.focal_length = 35;
  intrinsics_next_state.focus_distance = 1000;

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
    targets_states_subscriber_perception = n.subscribe<cinempc::MeasurementOut>(
        "/cinempc/perception_measurement_out", 1000, boost::bind(&readTargetStatePerceptionCallback, _1));
  }
  for (int i = 0; i < targets_names.size(); i++)
  {
    targets_states_subscribers_gt.push_back(n.subscribe<geometry_msgs::PoseStamped>(
        "airsim_node/" + targets_names.at(i) + "/get_pose", 1000, boost::bind(&readTargetStateCallback, _1, i)));
  }

  perception_meas_publisher = n.advertise<cinempc::MeasurementIn>("/cinempc/perception_measurement_in", 10);

  fpv_intrinsics_subscriber = n.subscribe<airsim_ros_pkgs::IntrinsicsCamera>(
      "/airsim_node/drone_1/set_intrinsics", 1000, boost::bind(&readIntrinsicsCallback, _1));

  low_level_control_publisher = n.advertise<cinempc::LowLevelControl>("cinempc/low_level_control", 10);

  estimation_in_publisher = n.advertise<cinempc::EstimationIn>("cinempc/estimation_in", 10);

  gimbal_rotation_subscriber = n.subscribe<airsim_ros_pkgs::GimbalAngleQuatCmd>(
      "/airsim_node/gimbal_angle_quat_cmd", 1000, boost::bind(&readGimbalOrientationCallback, _1));

  new_MPC_state_publisher = n.advertise<cinempc::MPCIncomingState>("cinempc/current_state", 10);

  cinempc_calculate_new_states_timer_ = n.createTimer(ros::Duration(mpc_dt), boost::bind(publishNewStateToMPC, _1, n));

  // init camera pose
  airsim_ros_pkgs::GimbalAngleQuatCmd msg;
  geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, 0, drone_start_yaw);
  msg.orientation = q;
  drone_pose.orientation = q;
  drone_pose_next_state.orientation = q;
  drone_pose_next_state = drone_pose;

  double yaw_step = 0, pitch_step = 0, focal_step = 0, ap_step;

  ros::Rate loop_rate(5);
  while (ros::ok() && !stop)
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
  logFile.close();
  int a = system("rm /home/pablo/Documents/journal/matlab/dolly_zoom/log_file.csv");
  string str1 = "cp " + logErrorFileName.str() + "  /home/pablo/Documents/journal/matlab/dolly_zoom/log_file.csv";
  a = system(str1.c_str());
  return 0;
}