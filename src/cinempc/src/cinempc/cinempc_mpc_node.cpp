#include "cinempc/cinempc_mpc_node.h"

std::vector<cinempc::TargetStateMPC> target_states;

cinempc::Constraints constraints;
cinempc::MPCResultPlotValues plot_values;

ros::Publisher results_pub;

// indexes to reference variables inside the `vars` vector
size_t x_start = 0;
size_t y_start = x_start + MPC_N;
size_t z_start = y_start + MPC_N;

size_t roll_gimbal_start = z_start + MPC_N;
size_t pitch_gimbal_start = roll_gimbal_start + MPC_N;
size_t yaw_gimbal_start = pitch_gimbal_start + MPC_N;

size_t focus_distance_start = yaw_gimbal_start + MPC_N;
size_t focal_length_start = focus_distance_start + MPC_N;
size_t aperture_start = focal_length_start + MPC_N;

size_t vel_x_start = aperture_start + MPC_N;
size_t vel_y_start = vel_x_start + MPC_N;
size_t vel_z_start = vel_y_start + MPC_N;

size_t vel_ang_x_start = vel_z_start + MPC_N;
size_t vel_ang_y_start = vel_ang_x_start + MPC_N - 1;
size_t vel_ang_z_start = vel_ang_y_start + MPC_N - 1;

size_t vel_focus_distance_start = vel_ang_z_start + MPC_N - 1;
size_t vel_focal_length_start = vel_focus_distance_start + MPC_N - 1;
size_t vel_aperture_start = vel_focal_length_start + MPC_N - 1;

size_t a_x_start = vel_aperture_start + MPC_N - 1;
size_t a_y_start = a_x_start + MPC_N - 1;
size_t a_z_start = a_y_start + MPC_N - 1;

// auxiliar method of MPC
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
  {
	A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++)
  {
	for (int i = 0; i < order; i++)
	{
	  A(j, i + 1) = A(j, i) * xvals(j);
	}
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

class FG_eval
{
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs)
  {
	this->coeffs = coeffs;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector &fg, const ADvector &vars)
  {
	// calculate errors
	int index_closest_target = 0;
	AD<double> distance_2D_target, minimum_distance_2D_target = 10000;
	for (int t = 0; t < MPC_N; t++)
	{
	  AD<double> Jp = 0, Jim = 0, JDoF = 0, JFoc = 0;
	  // J_DoF
	  AD<double> hyperfocal_distance_mms = cinempc::calculate_hyperfocal_distance_DoF<AD<double>>(
		  vars[focal_length_start + t], vars[aperture_start + t]);

	  AD<double> near_acceptable_distance = cinempc::calculate_near_distance_DoF<AD<double>>(
		  vars[focus_distance_start + t], ((AD<double>)hyperfocal_distance_mms / (AD<double>)1000),
		  (AD<double>)vars[focal_length_start + t] / (AD<double>)1000);

	  AD<double> far_acceptable_distance = cinempc::calculate_far_distance_DoF<AD<double>>(
		  vars[focus_distance_start + t], ((AD<double>)hyperfocal_distance_mms / (AD<double>)1000),
		  (AD<double>)vars[focal_length_start + t] / (AD<double>)1000);

	  AD<double> cost_near = CppAD::pow(near_acceptable_distance - constraints.dn_star, 2);
	  JDoF += constraints.weights.w_dn * (cost_near);

	  AD<double> cost_far = CppAD::pow(far_acceptable_distance - constraints.df_star, 2);
	  JDoF += constraints.weights.w_df * (cost_far);

	  // J_f
	  if (constraints.weights.w_focal > 0)
	  {
		AD<double> cost_foc = CppAD::pow(vars[focal_length_start + t] - constraints.focal_star, 2);
		JFoc += constraints.weights.w_focal * cost_foc;
	  }

	  fg[0] += JDoF + JFoc;	 // one time /camera
	  for (int j = 0; j < target_states.size(); j++)
	  {
		// calculate relative distances
		AD<double> relative_x_target = target_states.at(j).poses_top.at(t).position.x - (vars[x_start + t]);
		AD<double> relative_y_target = target_states.at(j).poses_top.at(t).position.y - (vars[y_start + t]);
		AD<double> relative_z_up_target = target_states.at(j).poses_top.at(t).position.z - (vars[z_start + t]);
		AD<double> relative_z_center_target = target_states.at(j).poses_center.at(t).position.z - (vars[z_start + t]);
		AD<double> relative_z_down_target = target_states.at(j).poses_bottom.at(t).position.z - (vars[z_start + t]);

		Eigen::Matrix<AD<double>, 3, 3> drone_R_target =
			cinempc::quat_to_R_matrix<AD<double>>(target_states.at(j).poses_top.at(t).orientation);

		Eigen::Matrix<AD<double>, 3, 3> new_drone_R = cinempc::RPY_to_R_matrix<AD<double>>(
			vars[roll_gimbal_start + t], vars[pitch_gimbal_start + t], vars[yaw_gimbal_start + t]);

		// J_im
		cinempc::Pixel<AD<double>> pixel_up_target = cinempc::calculate_image_position_from_3D<AD<double>>(
			vars[focal_length_start + t], relative_x_target, relative_y_target, relative_z_up_target, new_drone_R);

		cinempc::Pixel<AD<double>> pixel_down_target = cinempc::calculate_image_position_from_3D<AD<double>>(
			vars[focal_length_start + t], relative_x_target, relative_y_target, relative_z_down_target, new_drone_R);

		cinempc::Pixel<AD<double>> pixel_center_target = cinempc::calculate_image_position_from_3D<AD<double>>(
			vars[focal_length_start + t], relative_x_target, relative_y_target, relative_z_center_target, new_drone_R);

		AD<double> current_pixel_u_target = pixel_up_target.x;
		AD<double> current_pixel_v_target_up = pixel_up_target.y;
		AD<double> current_pixel_v_target_down = pixel_down_target.y;
		AD<double> current_pixel_v_target_center = pixel_center_target.y;
		if (constraints.weights.w_img_targets.at(j).x > 0)
		{
		  AD<double> weight = constraints.weights.w_img_targets.at(j).x;

		  AD<double> cost_pixel_u_target =
			  CppAD::pow(current_pixel_u_target - constraints.targets_im_top_star.at(j).x, 2);
		  if (cost_pixel_u_target < CppAD::pow(tolerance_pixel_x, 2))
		  {
			weight = weight * tolerance_reduce_weight;
		  }
		  Jim += weight * cost_pixel_u_target;
		}
		if (constraints.weights.w_img_targets.at(j).y_top > 0)
		{
		  AD<double> weight = constraints.weights.w_img_targets.at(j).y_top;
		  AD<double> cost_pixel_v_target_up =
			  CppAD::pow(current_pixel_v_target_up - constraints.targets_im_top_star.at(j).y, 2);
		  if (cost_pixel_v_target_up < CppAD::pow(tolerance_pixel_y, 2))
		  {
			weight = weight * tolerance_reduce_weight;
		  }
		  Jim += weight * cost_pixel_v_target_up;
		}

		if (constraints.weights.w_img_targets.at(j).y_center > 0)
		{
		  AD<double> weight = constraints.weights.w_img_targets.at(j).y_center;

		  AD<double> cost_pixel_v_target_center =
			  CppAD::pow(current_pixel_v_target_center - constraints.targets_im_center_star.at(j).y, 2);
		  if (cost_pixel_v_target_center < CppAD::pow(tolerance_pixel_y, 2))
		  {
			weight = weight * tolerance_reduce_weight;
		  }
		  Jim += weight * cost_pixel_v_target_center;
		}

		if (constraints.weights.w_img_targets.at(j).y_bottom > 0)
		{
		  AD<double> weight = constraints.weights.w_img_targets.at(j).y_bottom;

		  AD<double> cost_pixel_v_target_bottom =
			  CppAD::pow(current_pixel_v_target_down - constraints.targets_im_bottom_star.at(j).y, 2);
		  if (cost_pixel_v_target_bottom < CppAD::pow(tolerance_pixel_y, 2))
		  {
			weight = weight * tolerance_reduce_weight;
		  }
		  Jim += weight * cost_pixel_v_target_bottom;
		}

		// J_p
		distance_2D_target =
			cinempc::calculateDistanceTo2DPoint<AD<double>>(0, 0, relative_x_target, relative_y_target);
		if (distance_2D_target < minimum_distance_2D_target)
		{
		  minimum_distance_2D_target = distance_2D_target;
		  index_closest_target = j;
		}

		if (constraints.weights.w_d_targets.at(j) > 0)
		{
		  AD<double> cost_d_target = CppAD::pow((distance_2D_target - constraints.targets_d_star.at(j)), 2);
		  Jp += constraints.weights.w_d_targets.at(j) * cost_d_target;
		}
		Eigen::Matrix<AD<double>, 3, 3> new_drone_R_target;
		if (constraints.weights.w_R_targets.at(j) > 0)
		{
		  Eigen::Matrix<AD<double>, 3, 3> drone_R_star =
			  cinempc::quat_to_R_matrix<AD<double>>(constraints.targets_orientation_star.at(j));
		  new_drone_R_target = new_drone_R.transpose() * drone_R_target;

		  AD<double> cost_R_target = (new_drone_R_target - drone_R_star).norm();
		  Jp += constraints.weights.w_R_targets.at(j) * cost_R_target;
		}

		fg[0] += Jp + Jim;	// for each target

		if (t == 0 && j == 0)
		{
		  plot_values.Jp = Value(Jp);
		  plot_values.Jim = Value(Jim);
		  plot_values.JDoF = Value(JDoF);
		  plot_values.JFoc = Value(JFoc);
		  plot_values.dn = Value(near_acceptable_distance);
		  plot_values.df = Value(far_acceptable_distance);
		  plot_values.im_u = Value(current_pixel_u_target);
		  plot_values.im_v_up = Value(current_pixel_v_target_up);
		  plot_values.im_v_down = Value(current_pixel_v_target_down);
		  plot_values.im_v_center = Value(current_pixel_v_target_center);
		  plot_values.intrinsics_camera.focal_length = Value(vars[focal_length_start + 0]);
		  plot_values.intrinsics_camera.aperture = Value(vars[aperture_start + 0]);
		  plot_values.intrinsics_camera.focus_distance = Value(vars[focus_distance_start + 0]) * 100;

		  plot_values.d_gt = Value(distance_2D_target);
		  plot_values.relative_roll = Value(cinempc::R_matrix_to_RPY<AD<double>>(new_drone_R_target).roll);
		  plot_values.relative_yaw = Value(cinempc::R_matrix_to_RPY<AD<double>>(new_drone_R_target).yaw);
		  plot_values.relative_pitch = Value(cinempc::R_matrix_to_RPY<AD<double>>(new_drone_R_target).pitch);

		  if (log_costs)
		  {
			std::cout << "JDoF " << std::endl
					  << "--------- " << std::endl
					  << "   Dn:  " << near_acceptable_distance << std::endl
					  << "   dn_desired:  " << constraints.dn_star << std::endl
					  << "   Df:  " << far_acceptable_distance << std::endl
					  << "   df_desired:  " << constraints.df_star << std::endl;

			std::cout << "Jim " << std::endl
					  << "--------- " << std::endl
					  << "   current_pixel_u_target:  " << current_pixel_u_target << std::endl
					  << "   current_pixel_u_target_desired:  " << constraints.targets_im_top_star.at(j).x << std::endl
					  << "   current_pixel_v_up_target:  " << current_pixel_v_target_up << std::endl
					  << "   current_pixel_v_target_up_desired:  " << constraints.targets_im_top_star.at(j).y
					  << std::endl
					  << "   current_pixel_v_down_target:  " << current_pixel_v_target_down << std::endl
					  << "   current_pixel_v_target_down_desired:  " << constraints.targets_im_bottom_star.at(j).y
					  << std::endl
					  << "   current_pixel_v_center_target:  " << current_pixel_v_target_center << std::endl
					  << "   current_pixel_v_target_center_desired:  " << constraints.targets_im_center_star.at(j).y
					  << std::endl;

			std::cout << "JFoc " << std::endl
					  << "--------- " << std::endl
					  << "   current_focal:  " << vars[focal_length_start + t] << std::endl
					  << "   focal_desired:  " << constraints.focal_star << std::endl
					  << std::endl;

			std::cout << "Jp " << std::endl
					  << "--------- " << std::endl
					  << "   d_target:  " << distance_2D_target << std::endl
					  << "   d_target_desired:  " << constraints.targets_d_star.at(j) << std::endl
					  << "   roll_target:  " << cinempc::R_matrix_to_RPY<AD<double>>(new_drone_R_target).roll
					  << std::endl
					  << "   roll_target_desired:  "
					  << cinempc::quat_to_RPY<AD<double>>(constraints.targets_orientation_star.at(j)).roll << endl
					  << "   yaw_target:  " << cinempc::R_matrix_to_RPY<AD<double>>(new_drone_R_target).yaw << std::endl
					  << "   yaw_target_desired:  "
					  << cinempc::quat_to_RPY<AD<double>>(constraints.targets_orientation_star.at(j)).yaw << endl
					  << "   pitch_target:  " << cinempc::R_matrix_to_RPY<AD<double>>(new_drone_R_target).pitch
					  << std::endl
					  << "   pitch_target_desired:  "
					  << cinempc::quat_to_RPY<AD<double>>(constraints.targets_orientation_star.at(j)).pitch
					  << std::endl;

			std::cout << "RELATIVE DISTANCE " << std::endl
					  << "--------- " << std::endl
					  << "   relative_target_mpc_x_var:  " << relative_x_target << std::endl
					  << "   relative_target_mpc_y_var:  " << relative_y_target << std::endl
					  << "   relative_target_mpc_z_var:  " << relative_z_up_target << std::endl
					  << "   relative_target_mpc_z_down_var:  " << relative_z_down_target << std::endl;

			std::cout << "Cost:    " << Jp + Jim + JDoF << endl
					  << "   Jp:  " << Jp << endl
					  << "   Jim:  " << Jim << endl
					  << "   JDoF:  " << JDoF << endl
					  << "   JFoc:  " << JFoc << endl
					  << std::endl;
		  }
		}
	  }
	}
	///////////////////////////////
	////// setup constraints //////
	///////////////////////////////

	fg[MPC_N + x_start] = vars[x_start];
	fg[MPC_N + y_start] = vars[y_start];
	fg[MPC_N + z_start] = vars[z_start];

	fg[MPC_N + roll_gimbal_start] = vars[roll_gimbal_start];
	fg[MPC_N + pitch_gimbal_start] = vars[pitch_gimbal_start];
	fg[MPC_N + yaw_gimbal_start] = vars[yaw_gimbal_start];

	fg[MPC_N + focal_length_start] = vars[focal_length_start];
	fg[MPC_N + aperture_start] = vars[aperture_start];

	fg[MPC_N + vel_x_start] = vars[vel_x_start];
	fg[MPC_N + vel_y_start] = vars[vel_y_start];
	fg[MPC_N + vel_z_start] = vars[vel_z_start];

	// The rest of the constraints
	for (int t = 1; t < MPC_N; t++)
	{
	  // current state
	  AD<double> x0 = vars[x_start + t - 1];
	  AD<double> y0 = vars[y_start + t - 1];
	  AD<double> z0 = vars[z_start + t - 1];

	  AD<double> roll0 = vars[roll_gimbal_start + t - 1];
	  AD<double> pitch0 = vars[pitch_gimbal_start + t - 1];
	  AD<double> yaw0 = vars[yaw_gimbal_start + t - 1];

	  AD<double> focus0 = vars[focus_distance_start + t - 1];
	  AD<double> foc0 = vars[focal_length_start + t - 1];
	  AD<double> aperture0 = vars[aperture_start + t - 1];

	  AD<double> vel_x0 = vars[vel_x_start + t - 1];
	  AD<double> vel_y0 = vars[vel_y_start + t - 1];
	  AD<double> vel_z0 = vars[vel_z_start + t - 1];

	  AD<double> vel_ang_x0 = vars[vel_ang_x_start + t - 1];
	  AD<double> vel_ang_y0 = vars[vel_ang_y_start + t - 1];
	  AD<double> vel_ang_z0 = vars[vel_ang_z_start + t - 1];

	  AD<double> vel_focus0 = vars[vel_focus_distance_start + t - 1];
	  AD<double> vel_foc0 = vars[vel_focal_length_start + t - 1];
	  AD<double> vel_aperture0 = vars[vel_aperture_start + t - 1];

	  AD<double> a_x_0 = vars[a_x_start + t - 1];
	  AD<double> a_y_0 = vars[a_y_start + t - 1];
	  AD<double> a_z_0 = vars[a_z_start + t - 1];

	  // next state

	  AD<double> x1 = vars[x_start + t];
	  AD<double> y1 = vars[y_start + t];
	  AD<double> z1 = vars[z_start + t];

	  AD<double> roll1 = vars[roll_gimbal_start + t];
	  AD<double> pitch1 = vars[pitch_gimbal_start + t];
	  AD<double> yaw1 = vars[yaw_gimbal_start + t];

	  AD<double> focus1 = vars[focus_distance_start + t];
	  AD<double> foc1 = vars[focal_length_start + t];
	  AD<double> aperture1 = vars[aperture_start + t];

	  AD<double> vel_x1 = vars[vel_x_start + t];
	  AD<double> vel_y1 = vars[vel_y_start + t];
	  AD<double> vel_z1 = vars[vel_z_start + t];

	  fg[t] = cinempc::calculateDistanceTo2DPoint<AD<double>>(
		  x1, y1, target_states.at(index_closest_target).poses_top.at(t).position.x,
		  target_states.at(index_closest_target).poses_top.at(t).position.y);

	  // Setup the rest of the model constraints
	  fg[MPC_N + x_start + t] = x1 - (x0 + vel_x0 * mpc_dt);
	  fg[MPC_N + y_start + t] = y1 - (y0 + vel_y0 * mpc_dt);
	  fg[MPC_N + z_start + t] = z1 - (z0 + vel_z0 * mpc_dt);

	  // rotation dynamics
	  Eigen::Matrix<AD<double>, 3, 3> R = cinempc::RPY_to_R_matrix<AD<double>>(roll0, pitch0, yaw0);
	  Eigen::Matrix<AD<double>, 3, 3> R1;
	  Eigen::Matrix<AD<double>, 3, 1> angularVelocityVector(vel_ang_x0 + 0.000001, vel_ang_y0 + 0.000001,
															vel_ang_z0 + 0.000001);

	  Eigen::Matrix<AD<double>, 3, 3> wedgeFromAngularV =
		  cinempc::exp_wedge_operator<AD<double>>(angularVelocityVector * mpc_dt);
	  R1 = R * wedgeFromAngularV;

	  cinempc::RPY<AD<double>> RPYVector = cinempc::R_matrix_to_RPY<AD<double>>(R1);
	  fg[MPC_N + roll_gimbal_start + t] = roll1 - (RPYVector.roll);
	  fg[MPC_N + pitch_gimbal_start + t] = pitch1 - (RPYVector.pitch);
	  fg[MPC_N + yaw_gimbal_start + t] = yaw1 - (RPYVector.yaw);

	  fg[MPC_N + focus_distance_start + t] = focus1 - (focus0 + vel_focus0 * mpc_dt);
	  fg[MPC_N + focal_length_start + t] = foc1 - (foc0 + vel_foc0 * mpc_dt);
	  fg[MPC_N + aperture_start + t] = aperture1 - (aperture0 + vel_aperture0 * mpc_dt);

	  fg[MPC_N + vel_x_start + t] = vel_x1 - (vel_x0 + a_x_0 * mpc_dt);
	  fg[MPC_N + vel_y_start + t] = vel_y1 - (vel_y0 + a_y_0 * mpc_dt);
	  fg[MPC_N + vel_z_start + t] = vel_z1 - (vel_z0 + a_z_0 * mpc_dt);
	}
  }
};

void newStateReceivedCallback(const cinempc::MPCIncomingState::ConstPtr &msg)
{
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  target_states = msg->targets;

  constraints = msg->constraints;

  // number of independent variables
  size_t n_vars = MPC_N * 12 + 9 * (MPC_N - 1);	 // accelerations + constraints

  // Number of constraints
  size_t n_constraints = MPC_N * 13 - 1;

  // *** Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
  {
	vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = msg->drone_state.drone_pose.position.x;
  vars[y_start] = msg->drone_state.drone_pose.position.y;
  vars[z_start] = msg->drone_state.drone_pose.position.z;

  cinempc::RPY<double> rpy = cinempc::quat_to_RPY<double>(msg->drone_state.drone_pose.orientation);

  vars[roll_gimbal_start] = rpy.roll;
  vars[pitch_gimbal_start] = rpy.pitch;
  vars[yaw_gimbal_start] = rpy.yaw;

  vars[focus_distance_start] = msg->drone_state.intrinsics.focus_distance;
  vars[focal_length_start] = msg->drone_state.intrinsics.focal_length;
  vars[aperture_start] = msg->drone_state.intrinsics.aperture;

  vars[vel_x_start] = msg->drone_state.velocity.x;
  vars[vel_y_start] = msg->drone_state.velocity.y;
  vars[vel_z_start] = msg->drone_state.velocity.z;
  // *** Set lower and upper limits for the state variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values
  for (int i = 0; i < n_vars; i++)
  {
	double lowerbound, upperbound;
	if (i >= x_start && i < y_start)
	{
	  if (drone_moving)
	  {
		lowerbound = y_lowest;
		upperbound = y_highest;
	  }
	  else
	  {
		lowerbound = -0.0001;
		upperbound = 0.0001;
	  }
	}
	else if (i >= y_start && i < z_start)
	{
	  if (drone_moving)
	  {
		lowerbound = y_lowest;
		upperbound = y_highest;
	  }
	  else
	  {
		lowerbound = -0.0001;
		upperbound = 0.0001;
	  }
	}
	else if (i >= z_start && i < roll_gimbal_start)
	{
	  if (drone_moving && floor_constraints)
	  {
		lowerbound = -0.3;	// y_lowest;
		upperbound = msg->floor_pos - 0.2;
	  }
	  else if (!floor_constraints)
	  {
		lowerbound = z_lowest;
		upperbound = z_highest;
	  }
	  else
	  {
		lowerbound = -0.0001;
		upperbound = 0.0001;
	  }
	}

	else if (i >= roll_gimbal_start && i < pitch_gimbal_start)
	{
	  lowerbound = roll_lowest;
	  upperbound = roll_highest;
	}
	else if (i >= pitch_gimbal_start && i < yaw_gimbal_start)
	{
	  lowerbound = pitch_lowest;
	  upperbound = pitch_highest;
	}
	else if (i >= yaw_gimbal_start && i < focus_distance_start)
	{
	  lowerbound = yaw_lowest;
	  upperbound = yaw_highest;
	}
	else if (i >= focus_distance_start && i < focal_length_start)
	{
	  if (use_cineMPC)
	  {
		lowerbound = focus_lowest;
		upperbound = focus_highest;
	  }
	  else
	  {
		lowerbound = 1000;
		upperbound = 1000;
	  }
	}
	else if (i >= focal_length_start && i < aperture_start)
	{
	  if (use_cineMPC)
	  {
		lowerbound = foc_lowest;
		upperbound = foc_highest;
	  }
	  else
	  {
		lowerbound = 35;
		upperbound = 35;
	  }
	}
	else if (i >= aperture_start && i < vel_x_start)
	{
	  if (use_cineMPC)
	  {
		lowerbound = aperture_lowest;
		upperbound = aperture_highest;
	  }
	  else
	  {
		lowerbound = 20;
		upperbound = 20;
	  }
	}
	else if (i >= vel_x_start && i < vel_y_start)
	{
	  lowerbound = vel_x_lowest;
	  upperbound = vel_x_highest;
	}
	else if (i >= vel_y_start && i < vel_z_start)
	{
	  lowerbound = vel_y_lowest;
	  upperbound = vel_y_highest;
	}
	else if (i >= vel_z_start && i < vel_ang_x_start)
	{
	  lowerbound = vel_z_lowest;
	  upperbound = vel_z_highest;
	}
	else if (i >= vel_ang_x_start && i < vel_ang_y_start)
	{
	  lowerbound = vel_ang_x_lowest;
	  upperbound = vel_ang_x_highest;
	}
	else if (i >= vel_ang_y_start && i < vel_ang_z_start)
	{
	  lowerbound = vel_ang_y_lowest;
	  upperbound = vel_ang_y_highest;
	}
	else if (i >= vel_ang_z_start && i < vel_focal_length_start)
	{
	  lowerbound = vel_ang_z_lowest;
	  upperbound = vel_ang_z_highest;
	}
	else if (i >= vel_focus_distance_start && i < vel_focal_length_start)
	{
	  lowerbound = vel_focus_lowest;
	  upperbound = vel_focus_highest;
	}
	else if (i >= vel_focal_length_start && i < vel_aperture_start)
	{
	  if (use_cineMPC)
	  {
		lowerbound = vel_foc_lowest;
		upperbound = vel_foc_highest;
          }
           else
          {
		lowerbound = 0;
		upperbound = 0;
	  }
	}
	else if (i >= vel_aperture_start && i < a_x_start)
	{
	  if (use_cineMPC)
	  {
		lowerbound = vel_aperture_lowest;
		upperbound = vel_aperture_highest;
	  }
	  else
	  {
		lowerbound = 0;
		upperbound = 0;
	  }
	}
	else if (i >= a_x_start && i < a_y_start)
	{
	  lowerbound = a_x_lowest;
	  upperbound = a_x_highest;
	}
	else if (i >= a_y_start && i < a_z_start)
	{
	  lowerbound = a_y_lowest;
	  upperbound = a_y_highest;
	}
	else if (i >= a_z_start && i < n_vars)
	{
	  lowerbound = a_z_lowest;
	  upperbound = a_z_highest;
	}
	vars_lowerbound[i] = lowerbound;
	vars_upperbound[i] = upperbound;
  }

  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (int i = 0; i < n_constraints; i++)
  {
	constraints_lowerbound[i] = 0;
	constraints_upperbound[i] = 0;
  }

  // constraints for keeping the distance of security(fg[1-MPC_N])
  for (int i = 0; i < MPC_N - 1; i++)
  {
	constraints_lowerbound[i] = 2;
	constraints_upperbound[i] = 100000;
  }

  // initial state
  constraints_lowerbound[x_start + MPC_N - 1] = msg->drone_state.drone_pose.position.x;
  constraints_lowerbound[y_start + MPC_N - 1] = msg->drone_state.drone_pose.position.y;
  constraints_lowerbound[z_start + MPC_N - 1] = msg->drone_state.drone_pose.position.z;

  constraints_lowerbound[roll_gimbal_start + MPC_N - 1] = rpy.roll;
  constraints_lowerbound[pitch_gimbal_start + MPC_N - 1] = rpy.pitch;
  constraints_lowerbound[yaw_gimbal_start + MPC_N - 1] = rpy.yaw;

  constraints_lowerbound[focus_distance_start + MPC_N - 1] = msg->drone_state.intrinsics.focus_distance;
  constraints_lowerbound[focal_length_start + MPC_N - 1] = msg->drone_state.intrinsics.focal_length;
  constraints_lowerbound[aperture_start + MPC_N - 1] = msg->drone_state.intrinsics.aperture;

  constraints_lowerbound[vel_x_start + MPC_N - 1] = msg->drone_state.velocity.x;
  constraints_lowerbound[vel_y_start + MPC_N - 1] = msg->drone_state.velocity.y;
  constraints_lowerbound[vel_z_start + MPC_N - 1] = msg->drone_state.velocity.z;

  constraints_upperbound[x_start + MPC_N - 1] = msg->drone_state.drone_pose.position.x;
  constraints_upperbound[y_start + MPC_N - 1] = msg->drone_state.drone_pose.position.y;
  constraints_upperbound[z_start + MPC_N - 1] = msg->drone_state.drone_pose.position.z;

  constraints_upperbound[roll_gimbal_start + MPC_N - 1] = rpy.roll;
  constraints_upperbound[pitch_gimbal_start + MPC_N - 1] = rpy.pitch;
  constraints_upperbound[yaw_gimbal_start + MPC_N - 1] = rpy.yaw;

  constraints_upperbound[focus_distance_start + MPC_N - 1] = msg->drone_state.intrinsics.focus_distance;
  constraints_upperbound[focal_length_start + MPC_N - 1] = msg->drone_state.intrinsics.focal_length;
  constraints_upperbound[aperture_start + MPC_N - 1] = msg->drone_state.intrinsics.aperture;

  constraints_upperbound[vel_x_start + MPC_N - 1] = msg->drone_state.velocity.x;
  constraints_upperbound[vel_y_start + MPC_N - 1] = msg->drone_state.velocity.y;
  constraints_upperbound[vel_z_start + MPC_N - 1] = msg->drone_state.velocity.z;

  // object that computes objective and constraints

  Eigen::VectorXd xvals(30);
  Eigen::VectorXd yvals(30);

  auto coeffs = polyfit(xvals, yvals, 3);

  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time         1\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
										constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  if (!ok)
  {
	auto cost = solution.obj_value;

	plot_values.cost = cost;

	cinempc::MPCResult response_msg;

	response_msg.cost = cost;
	AD<double> vel_ang_x = solution.x[vel_ang_x_start];
	AD<double> vel_ang_y = solution.x[vel_ang_y_start];
	AD<double> vel_ang_z = solution.x[vel_ang_z_start];
	for (int i = 0; i < MPC_N; i++)
	{
	  cinempc::DroneAndCameraState state;
	  state.drone_pose.position.x = solution.x[x_start + i];
	  state.drone_pose.position.y = solution.x[y_start + i];
	  state.drone_pose.position.z = solution.x[z_start + i];

	  tf2::Quaternion quaternion_tf;
	  quaternion_tf.setRPY(solution.x[roll_gimbal_start + i], solution.x[pitch_gimbal_start + i],
						   solution.x[yaw_gimbal_start + i]);
	  quaternion_tf.normalize();

	  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quaternion_tf);

	  state.drone_pose.orientation = quat_msg;

	  state.intrinsics.focus_distance = solution.x[focus_distance_start + i];
	  state.intrinsics.focal_length = solution.x[focal_length_start + i];
	  state.intrinsics.aperture = solution.x[aperture_start + i];

	  state.velocity.x = solution.x[vel_x_start + i];
	  state.velocity.y = solution.x[vel_y_start + i];
	  state.velocity.z = solution.x[vel_z_start + i];

	  response_msg.mpc_n_states.push_back(state);
	}

	response_msg.plot_values = plot_values;
	results_pub.publish(response_msg);
  }
}

void restartSimulation(const std_msgs::Bool bool1)
{
  airsim_ros_pkgs::Takeoff srv;
  srv.request.waitOnLastTask = false;
  if (take_off_when_start)
  {
	service_take_off.call(srv);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cinempc_mpc");
  ros::NodeHandle n;

  service_take_off = n.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
  airsim_ros_pkgs::Takeoff srv;
  srv.request.waitOnLastTask = false;

  if (take_off_when_start)
  {
	service_take_off.call(srv);
  }

  ros::Subscriber restart_simulation =
	  n.subscribe<std_msgs::Bool>("cinempc/restart_simulation", 1000, restartSimulation);

  ros::Subscriber new_state_received_sub =
	  n.subscribe<cinempc::MPCIncomingState>("cinempc/current_state", 1000, newStateReceivedCallback);
  results_pub = n.advertise<cinempc::MPCResult>("cinempc/next_n_states", 100);
  ros::spin();
  return 0;
}
