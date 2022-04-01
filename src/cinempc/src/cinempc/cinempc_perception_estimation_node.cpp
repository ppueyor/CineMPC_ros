#include "cinempc/cinempc_perception_estimation_node.h"

using namespace std;
using namespace std::chrono;

std::vector<KalmanFilterEigen> kalman_filter_targets;

KalmanFilterEigen initializeKalmanFilterTarget()
{
  int n = kf_states;        // Number of states
  int m = kf_measurements;  // Number of measurements

  double dt_kf = kf_dt;  // Time step that we receive each image/measurement. We need to make it match with
                         // update_airsim_img_response_every_n_sec param of the sim

  Eigen::MatrixXd A(n, n);  // System dynamics matrix
  Eigen::MatrixXd C(m, n);  // Output matrix
  Eigen::MatrixXd Q(n, n);  // Process noise covariance
  Eigen::MatrixXd R(m, m);  // Measurement noise covariance
  Eigen::MatrixXd P(n, n);  // Estimate error covariance

  A << kf_a, 0, 0, dt_kf, 0, 0, 0, kf_a, 0, 0, dt_kf, 0, 0, 0, kf_a, 0, 0, dt_kf, 0, 0, 0, kf_a, 0, 0, 0, 0, 0, 0, kf_a,
      0, 0, 0, 0, 0, 0, kf_a;
  C << kf_c, 0, 0, 0, 0, 0, 0, kf_c, 0, 0, 0, 0, 0, 0, kf_c, 0, 0, 0;

  // Reasonable covariance matrices
  Q << kf_q, 0, 0, 0, 0, 0, 0, kf_q, 0, 0, 0, 0, 0, 0, kf_q, 0, 0, 0, 0, 0, 0, kf_q, 0, 0, 0, 0, 0, 0, kf_q, 0, 0, 0, 0,
      0, 0, kf_q;
  R << kf_r, 0, 0, 0, kf_r, 0, 0, 0, kf_r;

  P << kf_p, 0, 0, 0, 0, 0, 0, kf_p, 0, 0, 0, 0, 0, 0, kf_p, 0, 0, 0, 0, 0, 0, kf_p, 0, 0, 0, 0, 0, 0, kf_p, 0, 0, 0, 0,
      0, 0,
      kf_p;  // bigger than  Q and R.

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilterEigen init(dt_kf, A, C, Q, R, P, n, m);

  Eigen::VectorXd x0(n);
  double t = 0;
  x0 << kf_init_x, kf_init_y, kf_init_z, kf_init_vx, kf_init_vy, kf_init_vz;
  init.init(t, x0);

  return init;
}

const Eigen::MatrixXd getNewAMatrix(double dt_kf)
{
  int n = kf_states;        // Number of states
  Eigen::MatrixXd A(n, n);  // System dynamics matrix

  A << 1, 0, 0, dt_kf, 0, 0, 0, 1, 0, 0, dt_kf, 0, 0, 0, 1, 0, 0, dt_kf, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 1;

  return A;
}

bool predictNWorldTopPosesFromKF(cinempc::GetNNextTargetPoses::Request& req,
                                 cinempc::GetNNextTargetPoses::Response& res)
{
  int kf_steps_each_mpc = round(mpc_dt / kalman_filter_targets.at(req.target_index).get_dt());
  int kf_states = kf_steps_each_mpc * MPC_N;

  Eigen::MatrixXd new_states(kf_states, kalman_filter_targets.at(req.target_index).numberOfStates());
  new_states = kalman_filter_targets.at(req.target_index).predict(kf_states);

  std::vector<geometry_msgs::Pose> poses;

  for (int i = 0; i < kf_states; i += kf_steps_each_mpc)
  {
    geometry_msgs::Point target_vel;
    geometry_msgs::Pose target_pose;

    target_pose.position.x = new_states(i, 0);
    target_pose.position.y = new_states(i, 1);
    target_pose.position.z = new_states(i, 2);
    target_vel.x = new_states(i, 3);
    target_vel.y = new_states(i, 4);
    target_vel.z = new_states(i, 5);

    if (i == 0)
    {
      res.velocity_target_kf = target_vel;
    }

    if (!static_target)
    {
      target_pose.orientation =
          cinempc::predict_target_world_orientation_from_velocity<double>(target_vel.x, target_vel.y, target_vel.z);
    }
    else
    {
      target_pose.orientation = cinempc::RPY_to_quat<double>(0, 0, target_1_yaw_gt);
    }

    poses.push_back(target_pose);
  }
  res.poses_target = poses;
  return true;
}

void updateKalmanWithNewMeasure(const cinempc::EstimationIn::ConstPtr& kf_in_msg)
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
  ros::init(argc, argv, "cinempc_perception_estimation");

  ros::NodeHandle n;

  initializeTargets();

  ros::Subscriber KF_subscriber =
      n.subscribe<cinempc::EstimationIn>("cinempc/estimation_in", 1000, updateKalmanWithNewMeasure);

  ros::ServiceServer service =
      n.advertiseService<cinempc::GetNNextTargetPoses::Request, cinempc::GetNNextTargetPoses::Response>(
          "/cinempc/"
          "get_n_target_poses",
          boost::bind(&predictNWorldTopPosesFromKF, _1, _2));

  ros::Rate loop_rate(400);
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
}