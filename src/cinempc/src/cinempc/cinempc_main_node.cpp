#include "cinempc/cinempc_main_node.h"

using namespace std;
using namespace std::chrono;

cinempc::PlotValues plot_values;

float vel_x, vel_y, vel_z;
float focal_length = 35, focus_distance = 10000, aperture = 22;

int experiments = initial_experiment;

float sequence = 0;
bool stop = false, low_cost = false, first_log = true;

double yaw_gimbal = drone_start_yaw, pitch_gimbal = 0;

ros::Time start_log;
ros::Time start_measure;

std::stringstream logErrorFileName;
std::ofstream logFile;

cinempc::MeasurementIn perception_meas_in_msg;

geometry_msgs::Pose drone_pose_next_state;
geometry_msgs::Pose drone_pose_when_called;

geometry_msgs::Quaternion orientation_target_gt;

std::vector<float> times_vector, roll_vector, yaw_vector, pitch_vector, focal_length_vector, focus_distance_vector,
    aperture_vector;

static std::default_random_engine generator;

void logRPY(Eigen::Matrix<double, 3, 3> R, string name)
{
  cinempc::RPY<double> wRs2 = cinempc::R_matrix_to_RPY<double>(R);

  std::cout << name << std::endl
            << "--------- " << std::endl
            << "   R-P-Y:wRd  " << wRs2.roll << " pitch:   " << wRs2.pitch << " YAW: " << wRs2.yaw << std::endl;
}

void logPosition(geometry_msgs::Point pos, string name)
{
  double distance_2D_target =
      cinempc::calculateDistanceTo2DPoint<double>(drone_pose.position.x, drone_pose.position.y, pos.x, pos.y);
  std::cout << name << std::endl
            << "--------- " << std::endl
            << "   x  " << pos.x << " y:   " << pos.y << " z: " << pos.z << "  Distance: " << distance_2D_target
            << std::endl;
}

void initializePerceptionMsg()
{
  perception_meas_in_msg.depth = sensor_msgs::Image();
  perception_meas_in_msg.rgb = sensor_msgs::Image();
}

cinempc::TargetState fulfillRelativePosesFromWorldTop(geometry_msgs::Pose world_pose_top,
                                                      geometry_msgs::Pose drone_pose_now)
{
  cinempc::TargetState result;

  geometry_msgs::Pose world_pose_center, world_pose_bottom;
  geometry_msgs::Pose drone_pose_top, drone_pose_center, drone_pose_bottom;

  float target_z_center = world_pose_top.position.z + 0.5;  // target_height / 2;  // careful with Tuareg
  float target_z_bottom = world_pose_top.position.z + target_1_height;

  world_pose_center.position.x = world_pose_top.position.x;
  world_pose_center.position.y = world_pose_top.position.y;
  world_pose_center.position.z = target_z_center;
  world_pose_center.orientation = world_pose_top.orientation;

  world_pose_bottom.position.x = world_pose_top.position.x;
  world_pose_bottom.position.y = world_pose_top.position.y;
  world_pose_bottom.position.z = target_z_bottom;
  world_pose_bottom.orientation = world_pose_top.orientation;

  result.pose_top = cinempc::calculate_relative_pose_drone_target<double>(world_pose_top, drone_pose);
  result.pose_center = cinempc::calculate_relative_pose_drone_target<double>(world_pose_center, drone_pose);
  result.pose_bottom = cinempc::calculate_relative_pose_drone_target<double>(world_pose_bottom, drone_pose);

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
}

void readGimbalOrientationCallback(const airsim_ros_pkgs::GimbalAngleQuatCmd::ConstPtr& msg)
{
  drone_pose.orientation = msg->orientation;
  plot_values.drone_rot_gt = msg->orientation;
}

void readIntrinsicsCallback(const cinempc::IntrinsicsCamera::ConstPtr& msg)
{
  focal_length = msg->focal_length;
  aperture = msg->aperture;
  focus_distance = msg->focus_distance / 100;
}

void readTargetStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int target_index)
{
  geometry_msgs::Pose wTtop_measure;

  float target_x_bottom_gt = msg->pose.position.x;
  float target_y_bottom_gt = msg->pose.position.y;
  float target_z_bottom_gt = msg->pose.position.z;

  float target_z_top_gt = target_z_bottom_gt - target_1_height;

  // world
  wTtop_measure.position.x = target_x_bottom_gt;
  wTtop_measure.position.y = target_y_bottom_gt + target_1_width / 2;
  wTtop_measure.position.z = target_z_top_gt;
  wTtop_measure.orientation = cinempc::RPY_to_quat<double>(0, 0, target_1_yaw_gt);

  plot_values.targets_world_gt[target_index] = wTtop_measure.position;

  // plot
  geometry_msgs::Pose relative_pose = cinempc::calculate_relative_pose_drone_target<double>(wTtop_measure, drone_pose);

  if (!use_perception)
  {
    ros::Time end_measure = ros::Time::now();
    ros::Duration time_measure = end_measure - start_measure;
    double time_s = time_measure.toSec();
    cinempc::EstimationIn estimation_in_msg;
    estimation_in_msg.world_pose_target = wTtop_measure;
    estimation_in_msg.target_index = target_index;
    estimation_in_msg.time_s = time_s;
    estimation_in_msg.measure = true;
    estimation_in_publisher.publish(estimation_in_msg);
    start_measure = ros::Time::now();
  }
}

void readTargetVelocityCallback(const geometry_msgs::Point::ConstPtr& msg, int target_index)
{
  plot_values.v_target_gt = *msg;
  orientation_target_gt = cinempc::predict_target_world_orientation_from_velocity<double>(msg->x, msg->y, msg->z);
  plot_values.target_rot_gt = orientation_target_gt;
}

void readTargetStatePerceptionCallback(const cinempc::MeasurementOut::ConstPtr& msg)
{
  std::vector<geometry_msgs::Point> state;
  ros::Time end_measure = ros::Time::now();
  ros::Duration time_measure = end_measure - start_measure;
  double time_s = time_measure.toSec() / sim_speed;

  if (msg->targets_found > 0)
  {
    for (cinempc::TargetState target_state : msg->targets_state)
    {
      for (string target_class : targets_classes)
      {
        if (target_state.target_name.find(target_class) != std::string::npos)
        {
          geometry_msgs::Pose wTtop_measure = cinempc::calculate_drone_world_pose_from_relative<double>(
              msg->drone_state.drone_pose, target_state.pose_top, false);
          if (is_person)
          {
            wTtop_measure.position.z += 0.2;  // to match with face of person
          }

          cinempc::EstimationIn estimation_in_msg;
          estimation_in_msg.world_pose_target = wTtop_measure;
          estimation_in_msg.target_index = 0;
          estimation_in_msg.time_s = time_s;
          estimation_in_msg.measure = true;
          estimation_in_publisher.publish(estimation_in_msg);
        }
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

void restartSimulation()
{
  experiments++;

  if (experiments < number_of_experiments)
  {
    logFile.close();

    auto const now = std::chrono::system_clock::now();
    auto const in_time_t = std::chrono::system_clock::to_time_t(now);
    logErrorFileName.str("");

    logErrorFileName << project_folder << "logs/" << targets_names.at(0) << "/log" << experiments << ".csv";
    logFile.open(logErrorFileName.str());  // pitch,roll,yaw for every time stampm

    logFile << cinempc::plot_values(plot_values, true);

    start_log = ros::Time::now();
    start_measure = ros::Time::now();

    cinempc_calculate_new_states_timer_.stop();
    cinempc_calculate_new_states_timer_.start();

    // init camera pose
    airsim_ros_pkgs::GimbalAngleQuatCmd msg;
    geometry_msgs::Quaternion q = cinempc::RPY_to_quat<double>(0, 0, drone_start_yaw);
    msg.orientation = q;
    drone_pose.orientation = q;
    drone_pose_next_state.orientation = q;
    drone_pose_next_state = drone_pose;

    vel_x = 0, vel_y = 0, vel_z = 0;

    geometry_msgs::Pose initial_drone_pose;
    geometry_msgs::Point initial_point;

    initial_point.x = std::rand() % 7 - 5;
    initial_point.y = std::rand() % 2;

    initial_drone_pose.position = initial_point;
    initial_drone_pose.orientation = cinempc::RPY_to_quat<double>(0, 0, drone_start_yaw);

    std::cout << initial_drone_pose.position.x << endl;
    set_vehicle_pose_publisher.publish(initial_drone_pose);
    drone_pose.position = initial_point;

    // init drone pose
    sequence = 0;
    focal_length = 35, focus_distance = 10000, aperture = 22;

    first_log = true;
  }
}

void changeSeqCallback(const std_msgs::Float32::ConstPtr& msg)
{
  sequence = msg->data;
  std::cout << "Sequence:" << sequence << std::endl;
  plot_values.sequence = sequence;
  if (sequence == final_sequence)
  {
    std_msgs::Bool bool1;
    restart_simulation_publisher.publish(bool1);
    restartSimulation();
  }
  else if (sequence == 0.5)
  {
    geometry_msgs::Pose initial_drone_pose;
    geometry_msgs::Point initial_point;

    initial_point.x = std::rand() % 7 - 5;
    initial_point.y = std::rand() % 2;

    initial_drone_pose.position = initial_point;
    initial_drone_pose.orientation = cinempc::RPY_to_quat<double>(0, 0, drone_start_yaw);

    std::cout << initial_drone_pose.position.x << endl;
    set_vehicle_pose_publisher.publish(initial_drone_pose);
    drone_pose.position = initial_point;
  }
}

void stopReceivedCallback(const std_msgs::Bool& msg)
{
  stop = msg.data;
}
void rgbReceivedCallback(const sensor_msgs::Image& msg)
{
  if (use_perception)
  {
    perception_meas_in_msg.rgb = msg;
    if (perception_meas_in_msg.depth.height != 0)
    {
      perception_meas_in_msg.drone_state.drone_pose = drone_pose;
      perception_meas_in_msg.drone_state.intrinsics =
          cinempc::getInstrinscsMsg<float>(focal_length, focus_distance, aperture);
      perception_meas_publisher.publish(perception_meas_in_msg);
      initializePerceptionMsg();
    }
  }
}

void depthReceivedCallback(const sensor_msgs::Image& msg)
{
  if (use_perception)
  {
    perception_meas_in_msg.depth = msg;
    if (perception_meas_in_msg.rgb.height != 0)
    {
      perception_meas_in_msg.drone_state.drone_pose = drone_pose;
      perception_meas_in_msg.drone_state.intrinsics =
          cinempc::getInstrinscsMsg<float>(focal_length, focus_distance, aperture);
      perception_meas_publisher.publish(perception_meas_in_msg);
      initializePerceptionMsg();
    }
  }
}

void publishNewStateToMPC(const ros::TimerEvent& e, ros::NodeHandle n)
{
  if (sequence >= 1)
  {
    drone_pose_when_called = drone_pose;

    cinempc::MPCIncomingState msg_mpc_in;
    msg_mpc_in.drone_state.drone_pose.position.x = 0;
    msg_mpc_in.drone_state.drone_pose.position.y = 0;
    msg_mpc_in.drone_state.drone_pose.position.z = 0;

    msg_mpc_in.drone_state_global.drone_pose.position.x = drone_pose.position.x;
    msg_mpc_in.drone_state_global.drone_pose.position.y = drone_pose.position.y;
    msg_mpc_in.drone_state_global.drone_pose.position.z = drone_pose.position.z;

    msg_mpc_in.drone_state.velocity.x = vel_x;
    msg_mpc_in.drone_state.velocity.y = vel_y;
    msg_mpc_in.drone_state.velocity.z = vel_z;

    geometry_msgs::Quaternion q = cinempc::RPY_to_quat<double>(0, 0, 0);
    msg_mpc_in.drone_state.drone_pose.orientation = q;
    msg_mpc_in.floor_pos = -drone_pose.position.z;
    msg_mpc_in.drone_state.intrinsics = cinempc::getInstrinscsMsg<float>(focal_length, focus_distance, aperture);

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
      ros::ServiceClient service_get_N_poses = n.serviceClient<cinempc::GetNNextTargetPoses>(
          "/cinempc/"
          "get_n_target_poses");

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
          if (t == 0)
          {
            plot_values.targets_world_kf[target] = world_pose_top_kf.position;
            plot_values.v_target_kf = srv_get_poses.response.velocity_target_kf;
          }

          plot_values.target_rot_perception = world_pose_top_kf.orientation;

          cinempc::TargetState target_state =
              fulfillRelativePosesFromWorldTop(world_pose_top_kf, drone_pose_when_called);

          msg_mpc_in.targets.at(target).poses_top.at(t) = target_state.pose_top;
          msg_mpc_in.targets.at(target).poses_center.at(t) = target_state.pose_center;
          msg_mpc_in.targets.at(target).poses_bottom.at(t) = target_state.pose_bottom;
        }
      }
    }
    ros::ServiceClient service_get_user_constraints = n.serviceClient<cinempc::GetUserConstraints>(
        "/cinempc/"
        "user_node/"
        "get_"
        "constraints");
    cinempc::GetUserConstraints srv;
    srv.request.targets_relative = msg_mpc_in.targets;
    srv.request.world_rotations_target = world_rotations;
    srv.request.drone_pose = drone_pose_when_called;
    srv.request.sequence = sequence;
    if (service_get_user_constraints.call(srv))
    {
      msg_mpc_in.constraints = srv.response.contraints;
      new_MPC_state_publisher.publish(msg_mpc_in);
      plot_values.constraints = srv.response.contraints;
    }
    if (!first_log)
    {
      logFile << cinempc::plot_values(plot_values, false);
    }
    else
    {
      first_log = false;
    }
  }
}

void mpcResultCallback(const cinempc::MPCResult::ConstPtr& msg)
{
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

  if (msg->cost >= 100 || msg->cost == 0)
  {
    low_cost = false;
    roll_vector.insert(roll_vector.begin(), cinempc::quat_to_RPY<double>(drone_pose_when_called.orientation).roll);
    pitch_vector.insert(pitch_vector.begin(), cinempc::quat_to_RPY<double>(drone_pose_when_called.orientation).pitch);
    yaw_vector.insert(yaw_vector.begin(), cinempc::quat_to_RPY<double>(drone_pose_when_called.orientation).yaw);
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

      world_T_result =
          cinempc::calculate_drone_world_pose_from_relative<double>(drone_pose_when_called, cine_mpc_result.drone_pose);
      cinempc::RPY<double> rpy = cinempc::quat_to_RPY<double>(world_T_result.orientation);

      roll_vector.insert(roll_vector.begin() + index_mpc, rpy.roll);
      yaw_vector.insert(yaw_vector.begin() + index_mpc, rpy.yaw);
      pitch_vector.insert(pitch_vector.begin() + index_mpc, rpy.pitch);

      geometry_msgs::Point path_point(world_T_result.position);
      pathMPC.push_back(path_point);

      if (index_mpc == 1)
      {
        drone_pose_next_state = world_T_result;
        vel_x = cine_mpc_result.velocity.x;
        vel_y = cine_mpc_result.velocity.y;
        vel_z = cine_mpc_result.velocity.z;
      }

      geometry_msgs::Pose dTvel;
      dTvel.position = cine_mpc_result.velocity;
      dTvel.orientation = world_T_result.orientation;
      geometry_msgs::Pose wTvel =
          cinempc::calculate_drone_world_pose_from_relative<double>(drone_pose_when_called, dTvel, true);

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

    low_level_control_msg.move_on_path_msg.vel = max_vel;
    low_level_control_msg.move_on_path_msg.timeout = 10;
    low_level_control_msg.move_on_path_msg.rads_yaw = cinempc::quat_to_RPY<double>(drone_pose.orientation).yaw;
    low_level_control_msg.move_on_path_msg.positions = pathMPC;

    low_level_control_publisher.publish(low_level_control_msg);
  }
  else
  {
    low_cost = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_main");

  auto const now = std::chrono::system_clock::now();
  auto const in_time_t = std::chrono::system_clock::to_time_t(now);

  logErrorFileName << project_folder << "logs/" << targets_names.at(0) << "/log" << experiments << ".csv";
  logFile.open(logErrorFileName.str());  // pitch,roll,yaw for every time stamp

  logFile << cinempc::plot_values(plot_values, true);

  ros::NodeHandle n;

  start_log = ros::Time::now();
  start_measure = ros::Time::now();

  ros::Subscriber new_drone_state_received_sub = n.subscribe(get_drone_position_topic, 1000, readDroneStateCallback);
  ros::Subscriber new_rgb_received_sub = n.subscribe(image_rgb_topic, 1000, rgbReceivedCallback);
  ros::Subscriber new_depth_received_sub = n.subscribe(image_depth_topic, 1000, depthReceivedCallback);
  ros::Subscriber fpv_intrinsics_subscriber =
      n.subscribe<cinempc::IntrinsicsCamera>(set_intrinsics_topic, 1000, boost::bind(&readIntrinsicsCallback, _1));
  ros::Subscriber gimbal_rotation_subscriber = n.subscribe<airsim_ros_pkgs::GimbalAngleQuatCmd>(
      get_drone_orientation_topic, 1000, boost::bind(&readGimbalOrientationCallback, _1));

  std::vector<ros::Subscriber> targets_states_subscribers_gt = {}, target_velocities_subscribers_gt = {};
  ros::Subscriber targets_states_subscriber_perception;

  set_vehicle_pose_publisher = n.advertise<geometry_msgs::Pose>(set_drone_pose_topic, 10);

  ros::Subscriber change_sequence_sub = n.subscribe("cinempc/sequence", 1000, changeSeqCallback);
  ros::Subscriber mpc_result_n_steps_sub = n.subscribe("cinempc/next_n_states", 1000, mpcResultCallback);
  ros::Subscriber stop_signal_received_sub = n.subscribe("/cinempc/stop", 1000, stopReceivedCallback);
  restart_simulation_publisher = n.advertise<std_msgs::Bool>("/cinempc/restart_simulation", 10);
  perception_meas_publisher = n.advertise<cinempc::MeasurementIn>("/cinempc/perception_measurement_in", 10);
  low_level_control_publisher = n.advertise<cinempc::LowLevelControl>("/cinempc/low_level_control", 1000);
  estimation_in_publisher = n.advertise<cinempc::EstimationIn>("cinempc/estimation_in", 10);
  new_MPC_state_publisher = n.advertise<cinempc::MPCIncomingState>("cinempc/current_state", 10);

  airsim_ros_pkgs::Takeoff srv;
  srv.request.waitOnLastTask = false;
  // service_take_off.call(srv);

  if (use_perception)
  {
    targets_states_subscriber_perception = n.subscribe<cinempc::MeasurementOut>(
        "/cinempc/perception_measurement_out", 1000, boost::bind(&readTargetStatePerceptionCallback, _1));
  }
  for (int i = 0; i < targets_names.size(); i++)
  {
    targets_states_subscribers_gt.push_back(n.subscribe<geometry_msgs::PoseStamped>(
        "airsim_node/" + targets_names.at(i) + "/get_pose", 1000, boost::bind(&readTargetStateCallback, _1, i)));

    target_velocities_subscribers_gt.push_back(
        n.subscribe<geometry_msgs::Point>("airsim_node/" + targets_names.at(i) + "/current_speed", 1000,
                                          boost::bind(&readTargetVelocityCallback, _1, i)));
  }

  cinempc_calculate_new_states_timer_ = n.createTimer(ros::Duration(mpc_dt), boost::bind(publishNewStateToMPC, _1, n));

  // init camera pose
  airsim_ros_pkgs::GimbalAngleQuatCmd msg;
  geometry_msgs::Quaternion q = cinempc::RPY_to_quat<double>(0, 0, drone_start_yaw);
  msg.orientation = q;
  drone_pose.orientation = q;
  drone_pose_next_state.orientation = q;
  drone_pose_next_state = drone_pose;

  ros::Rate loop_rate(100);
  bool called = false;
  while (ros::ok() && !stop)
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
  logFile.close();
  string delete_command = "rm " + project_folder + targets_names.at(0) + "/log_file.csv";
  int a = system(delete_command.c_str());
  string str1 =
      "cp " + logErrorFileName.str() + "  " + project_folder + "matlab/" + targets_names.at(0) + "/log_file.csv";
  a = system(str1.c_str());
  return 0;
}