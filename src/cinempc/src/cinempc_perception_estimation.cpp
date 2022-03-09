#include "cinempc_perception_estimation.h"

using namespace std;
using namespace std::chrono;

std::vector<KalmanFilterEigen> kalman_filter_targets;

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

bool predictNWorldTopPosesFromKF(cinempc::GetNNextTargetPoses::Request& req,
                                 cinempc::GetNNextTargetPoses::Response& res)
{
  Eigen::MatrixXd new_states(req.mpc_N, kalman_filter_targets.at(req.target_index).numberOfStates());

  double kf_time_each_mpc = req.mpc_dt / kalman_filter_targets.at(req.target_index).get_dt();
  new_states = kalman_filter_targets.at(req.target_index).predict(kf_time_each_mpc, MPC_N);

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
  }
  res.poses_target = poses;
  return true;
}

void updateKalmanWithNewMeasure(const cinempc::KFIn::ConstPtr& kf_in_msg)
{
  if (kf_in_msg->measure)
  {
    Eigen::VectorXd measurement(kalman_filter_targets.at(kf_in_msg->target_index).numberOfMeasurements());
    measurement(0) = kf_in_msg->world_pose_target.position.x;
    measurement(1) = kf_in_msg->world_pose_target.position.y;
    measurement(2) = kf_in_msg->world_pose_target.position.z;

    Eigen::MatrixXd A = getNewAMatrix(kf_in_msg->time_s);
    kalman_filter_targets.at(kf_in_msg->target_index).update(measurement, kf_in_msg->time_s, A);
  }
  else
  {
    Eigen::MatrixXd A = getNewAMatrix(kf_in_msg->time_s);
    kalman_filter_targets.at(kf_in_msg->target_index).update(kf_in_msg->time_s, A);
  }

  // Eigen::VectorXd new_state(kalman_filter_targets.at(kf_in_msg->target_index).numberOfStates());

  //  new_state = kalman_filter_targets.at(kf_in_msg->target_index).state();

  //   geometry_msgs::Point new_position;
  //   geometry_msgs::Point new_velocity;

  //   new_position.x = new_state(0);
  //   new_position.y = new_state(1);
  //   new_position.z = new_state(2);

  //   new_velocity.x = new_state(3);
  //   new_velocity.y = new_state(4);
  //   new_velocity.z = new_state(5);

  //   std::vector<geometry_msgs::Point> vector;
  //   vector.push_back(new_position);
  //   vector.push_back(new_velocity);

  //   return vector;
}

void initializeTargets()
{
  for (int i = 0; i < targets_names.size(); i++)
  {
    KalmanFilterEigen kf = initializeKalmanFilterTarget();
    kalman_filter_targets.push_back(kf);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cinempc_main");

  ros::NodeHandle n;

  initializeTargets();

  KF_subscriber = n.subscribe<cinempc::KFIn>("cinempc/kf_in", 1000, updateKalmanWithNewMeasure);

  ros::ServiceServer service =
      n.advertiseService<cinempc::GetNNextTargetPoses::Request, cinempc::GetNNextTargetPoses::Response>(
          "/cinempc/"
          "get_N_target_poses",
          boost::bind(&predictNWorldTopPosesFromKF, _1, _2));

  ros::Rate loop_rate(200);
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
}