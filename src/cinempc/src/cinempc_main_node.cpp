#include "cinempc_main_node.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

// Current position of drone
std::vector<geometry_msgs::Pose> targets_poses;
string errors;

geometry_msgs::Pose target1_pose;

float focal_length = 35, focus_distance = 10000, aperture = 20;

int index_splines = 0;
bool noise = true;
float sequence = 0;
int change_sequence_index = 0;
double steps_each_dt = 20;
double interval = dt / steps_each_dt;
double freq_loop = 1 / interval;
std::stringstream logErrorFileName;
std::ofstream errorFile;

cinempc::PerceptionMsg perception_msg;

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

  target1_pose = cinempc::calculate_relative_pose_drone_person<double>(targets_poses.at(i), drone_pose);
}

void readTargetStatePerceptionCallback(const geometry_msgs::Pose& msg)
{
  float target_x_perception = msg.position.x;  // + generateNoise(0, 0.04);
  float target_y_perception = msg.position.y;  // + generateNoise(0, 0.04);
  float target_z_perception = msg.position.z;  // + generateNoise(0, 0.04);
  // std::cout << "target_x_perception: " << target_x_perception << "  target_y_perception: " << target_y_perception
  //<< "  target_z_perception: " << target_z_perception << std::endl;

  float error_x = abs(target_x_perception - (targets_poses.at(0).position.x - drone_pose.position.x));
  float error_y = abs(target_y_perception - (targets_poses.at(0).position.y - drone_pose.position.y));
  float error_z = abs(target_z_perception - (targets_poses.at(0).position.z - drone_pose.position.z));

  std::cout << "error_x: " << error_x << "  error_y: " << error_y << "  error_z: " << error_z << std::endl;

  errorFile << error_x << "," << error_y << "," << error_z << "\n";
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

    // std::cout << "Point:" << path_point.x << "  " << path_point.y << "   " << path_point.z << std::endl;

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
  cinempc::MPCIncomingState msg;
  msg.drone_state.drone_pose.position.x = 0;  // drone_pose.position.x;
  msg.drone_state.drone_pose.position.y = 0;  // drone_pose.position.y;
  msg.drone_state.drone_pose.position.z = 0;  // drone_pose.position.z;

  geometry_msgs::Quaternion q = cinempc::RPYToQuat<double>(0, 0, 0);
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
        ros::ServiceClient service_get_user_constraints = n.serviceClient<cinempc::GetUserConstraints>("/cinempc/"
                                                                                                       "user_node/"
                                                                                                       "get_"
                                                                                                       "constraints");
        cinempc::GetUserConstraints srv;
        srv.request.targets = msg.targets;
        srv.request.sequence = 1;

        if (service_get_user_constraints.call(srv))
        {
          msg.constraints = srv.response.contraints;
          new_state_publisher.publish(msg);
        }
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

  auto const now = std::chrono::system_clock::now();
  auto const in_time_t = std::chrono::system_clock::to_time_t(now);
  logErrorFileName << "/home/pablo/Desktop/AirSim_update/AirSim_ros/ros/src/cinempc/logs/error_pos_"
                   << std::put_time(std::localtime(&in_time_t), "%d_%m_%Y-%H_%M_%S") << ".csv";
  errorFile.open(logErrorFileName.str());  // pitch,roll,yaw for every time stamp
  errorFile << "Error x"
            << ","
            << "Error y"
            << ","
            << "Error z";

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

  ros::Subscriber new_rgb_received = n.subscribe("/airsim_node/drone_1/Scene", 1000, rgbReceivedCallback);
  ros::Subscriber new_depth_received = n.subscribe("/airsim_node/drone_1/DepthVis", 1000, depthReceivedCallback);

  std::vector<ros::Subscriber> targets_states_subscribers = {};
  for (int i = 0; i < targets_names.size(); i++)
  {
    targets_states_subscribers.push_back(n.subscribe<geometry_msgs::PoseStamped>(
        "airsim_node/" + targets_names.at(i) + "/get_pose", 1000, boost::bind(&readTargetStateCallback, _1, i)));
  }

  ros::Subscriber target_state_perception_subscriber =
      n.subscribe("/cinempc/target_pose_perception", 1000, readTargetStatePerceptionCallback);

  fpv_intrinsics_publisher = n.advertise<airsim_ros_pkgs::IntrinsicsCamera>("/airsim_node/drone_1/set_intrinsics", 10);

  perception_publisher = n.advertise<cinempc::PerceptionMsg>("/cinempc/perception", 10);

  fpv_intrinsics_publisher.publish(getInstrinscsMsg(focal_length, focus_distance, aperture));

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
  return 0;
}
