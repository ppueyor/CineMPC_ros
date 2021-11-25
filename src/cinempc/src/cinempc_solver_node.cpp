#include <cppad/cppad.hpp>

#include "cinempc_solver_node.h"
#include "cppad/ipopt/solve.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui.hpp"
//#include "opencv2/xfeatures2d.hpp"

using CppAD::AD;
using namespace Eigen;
using namespace std;
// Set the timestep length and duration

std::vector<cinempc::PersonStateMPC> target_states;

struct Pixel_MPC
{
  AD<double> x;
  AD<double> y;
};

AD<double> current_Ji, current_Jp, current_JDoF, dn_plot, df_plot, roll_plot, roll_plot_d, pitch_plot, pitch_plot_d,
	yaw_plot, yaw_plot_d, hyper_plot;
std::vector<Pixel_MPC> current_pixel_target_up_plot, current_pixel_target_down_plot, current_pixel_target_up_d_plot,
	current_pixel_target_down_d_plot;
std::vector<AD<double>> d_target_plot;
cinempc::Constraints constraints;

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

// mms
AD<double> hyperFocalDistance(AD<double> focal_length, AD<double> aperture)
{
  AD<double> div = 0.0;
  AD<double> term1 = (AD<double>)(CppAD::pow(focal_length, 2) + 0.00001);
  AD<double> term2 = (AD<double>)((aperture * circle_confusion) + 0.00001);
  div = (AD<double>)term1 / (AD<double>)term2;

  // std::cout << "current_distance:" << currentDistance << std::endl;
  /*std::cout << "aperture  " << aperture << "terms:" << term1 << "  " << term2
   << "   " << div << std::endl;*/

  return div + focal_length;
}

AD<double> nearAcceptableDistance(AD<double> focus_distance, AD<double> hyperfocal_distance, AD<double> focal_length)
{
  AD<double> term1 = focus_distance * (hyperfocal_distance - focal_length);
  AD<double> term2 = hyperfocal_distance + focus_distance - 2 * focal_length;

  AD<double> div = (AD<double>)term1 / (AD<double>)term2;

  // std::cout << "current_distance:" << currentDistance << std::endl;
  /*std::cout  << "hyperfocal_distance:  "<< hyperfocal_distance
   << "    focal_length:   "<< focal_length <<
   "terms:"<< term1 <<"  "<<term2 <<"   " << div << std::endl;*/
  return (div);
}

AD<double> farAcceptableDistance(AD<double> focus_distance, AD<double> hyperfocal_distance, AD<double> focal_length)
{
  AD<double> term1 = focus_distance * (hyperfocal_distance - focal_length);
  AD<double> term2 = hyperfocal_distance - focus_distance;

  AD<double> div = (AD<double>)term1 / (AD<double>)term2;

  // std::cout << "current_distance:" << currentDistance << std::endl;
  /*std::cout  << "hyperfocal_distance:  "<< hyperfocal_distance
   << "    focal_length:   "<< focal_length <<
   "terms:"<< term1 <<"  "<<term2 <<"   " << div << std::endl;*/
  return (div);
}

// from slides of AROB
Eigen::Matrix<AD<double>, 3, 3> wedgeOperator(Eigen::Matrix<AD<double>, 3, 1> M)
{
  Eigen::Matrix<AD<double>, 3, 3> Wedged;
  Wedged(0, 0) = 0;
  Wedged(0, 1) = -M(2);
  Wedged(0, 2) = M(1);
  Wedged(1, 0) = M(2);
  Wedged(1, 1) = 0;
  Wedged(1, 2) = -M(0);
  Wedged(2, 0) = -M(1);
  Wedged(2, 1) = M(0);
  Wedged(2, 2) = 0;

  return (Wedged);
}

// from slides of AROB
Eigen::Matrix<AD<double>, 3, 3> convertMatrixToDiff(Eigen::Matrix<double, 3, 3> M)
{
  Eigen::Matrix<AD<double>, 3, 3> R_diff;
  R_diff(0, 0) = M(0, 0);
  R_diff(0, 1) = M(0, 1);
  R_diff(0, 2) = M(0, 2);
  R_diff(1, 0) = M(1, 0);
  R_diff(1, 1) = M(1, 1);
  R_diff(1, 2) = M(1, 2);
  R_diff(2, 0) = M(2, 0);
  R_diff(2, 1) = M(2, 1);
  R_diff(2, 2) = M(2, 2);

  return (R_diff);
}

Eigen::Matrix<AD<double>, 3, 3> expWedgeOperator(Eigen::Matrix<AD<double>, 3, 1> M)
{
  Eigen::Matrix<AD<double>, 3, 3> I = Eigen::Matrix<AD<double>, 3, 3>::Identity(3, 3);

  Eigen::Matrix<AD<double>, 3, 1> RotationAxis;
  AD<double> rotationAngle;
  if (M(0) == 0 && M(1) == 0 && M(2) == 0)
  {
	rotationAngle = 0;
	RotationAxis = Eigen::Matrix<AD<double>, 3, 1>::Zero(3, 1);
  }
  else
  {
	rotationAngle = M.norm();
	RotationAxis = M / rotationAngle;
  }

  Eigen::Matrix<AD<double>, 3, 3> WedgedMatrix = wedgeOperator(RotationAxis);

  Eigen::Matrix<AD<double>, 3, 3> Result;
  Result = I + sin(rotationAngle) * WedgedMatrix + (1 - cos(rotationAngle)) * (WedgedMatrix * WedgedMatrix);

  return (Result);
}

Pixel_MPC readPixel(AD<double> focal_length_mm, AD<double> relative_position_x_m, AD<double> relative_position_y_m,
					AD<double> relative_position_z_m, Eigen::Matrix<AD<double>, 3, 3> R)
{
  // Read comment to understand change of variables
  AD<double> distance_y_mm = relative_position_y_m * 1000;	// ms to mms
  AD<double> distance_z_mm = relative_position_z_m * 1000;
  AD<double> distance_x_mm = relative_position_x_m * 1000;

  Eigen::Matrix<AD<double>, 3, 1> P(distance_x_mm, distance_y_mm, distance_z_mm);
  Eigen::Matrix<AD<double>, 3, 1> rPs = R.transpose() * P;

  Eigen::Matrix<AD<double>, 3, 1> vectorPixels(rPs(1), rPs(2), rPs(0));

  AD<double> x0_mm = sensor_width_mm / 2;
  AD<double> y0_mm = sensor_height_mm / 2;

  AD<double> x_mm = x0_mm + ((focal_length_mm * vectorPixels(0)) / vectorPixels(2));
  AD<double> y_mm = y0_mm + ((focal_length_mm * vectorPixels(1)) / vectorPixels(2));

  AD<double> xPixels_px = x_mm * picture_width_px / sensor_width_mm;

  AD<double> yPixels_px = y_mm * picture_height_px / sensor_height_mm;

  Pixel_MPC p;
  p.x = xPixels_px;
  sensor_width_mm / 2 + ((focal_length_mm * vectorPixels(0)) / vectorPixels(2)) * picture_width_px / sensor_width_mm;
  p.y = yPixels_px;

  return (p);
}

void logRPY(Eigen::Matrix<AD<double>, 3, 3> R, string name)
{
  cinempc::RPY<AD<double>> wRs2 = cinempc::RMatrixtoRPY<AD<double>>(R);

  std::cout << name << std::endl
			<< "--------- " << std::endl
			<< "   R-P-Y:wRd  " << wRs2.roll << " pitch:   " << wRs2.pitch << " YAW: " << wRs2.yaw << std::endl;
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
	int seq = 0, closest_target_index = 0;
	AD<double> distance_2D_target, minimum_distance_2D_target = 10000;
	for (int t = 0; t < MPC_N; t++)
	{
	  AD<double> Jp = 0, Jim = 0, JDoF = 0;
	  for (int j = 0; j < target_states.size(); j++)
	  {
		// calculate relative distances
		AD<double> relative_x_target = target_states.at(j).poses_up.at(t).position.x - (vars[x_start + t]);
		AD<double> relative_y_target = target_states.at(j).poses_up.at(t).position.y - (vars[y_start + t]);
		AD<double> relative_z_up_target = target_states.at(j).poses_up.at(t).position.z - (vars[z_start + t]);
		AD<double> relative_z_down_target = target_states.at(j).poses_down.at(t).position.z - (vars[z_start + t]);

		Eigen::Matrix<AD<double>, 3, 3> drone_R_target =
			cinempc::quatToRMatrix<AD<double>>(target_states.at(j).poses_up.at(t).orientation);

		Eigen::Matrix<AD<double>, 3, 3> new_drone_R = cinempc::RPYtoRMatrix<AD<double>>(
			vars[roll_gimbal_start + t], vars[pitch_gimbal_start + t], vars[yaw_gimbal_start + t]);

		// Cost_DoF
		AD<double> hyperfocal_distance_mms = hyperFocalDistance(vars[focal_length_start + t], vars[aperture_start + t]);

		AD<double> near_acceptable_distance = nearAcceptableDistance(
			vars[focus_distance_start + t], ((AD<double>)hyperfocal_distance_mms / (AD<double>)1000),
			(AD<double>)vars[focal_length_start + t] / (AD<double>)1000);

		AD<double> far_acceptable_distance = farAcceptableDistance(
			vars[focus_distance_start + t], ((AD<double>)hyperfocal_distance_mms / (AD<double>)1000),
			(AD<double>)vars[focal_length_start + t] / (AD<double>)1000);

		AD<double> cost_near = CppAD::pow(near_acceptable_distance - constraints.dn_star, 2);
		JDoF += constraints.weights.w_dn * (cost_near);

		AD<double> cost_far = CppAD::pow(far_acceptable_distance - constraints.df_star, 2);
		JDoF += constraints.weights.w_df * (cost_far);

		// Cost_img
		Pixel_MPC pixel_up_target = readPixel(vars[focal_length_start + t],	 // head
											  relative_x_target, relative_y_target, relative_z_up_target, new_drone_R);

		Pixel_MPC pixel_down_target = readPixel(vars[focal_length_start + t], relative_x_target, relative_y_target,
												relative_z_down_target, new_drone_R);

		AD<double> current_pixel_u_target = pixel_up_target.x;
		AD<double> current_pixel_v_target_up = pixel_up_target.y;
		AD<double> current_pixel_v_target_down = pixel_down_target.y;
		if (constraints.weights.w_img_targets.at(j).x > 0)
		{
		  AD<double> cost_pixel_u_target =
			  CppAD::pow(current_pixel_u_target - constraints.targets_im_up_star.at(j).x, 2);
		  Jim += constraints.weights.w_img_targets.at(j).x * cost_pixel_u_target;
		}
		if (constraints.weights.w_img_targets.at(j).y > 0)
		{
		  AD<double> cost_pixel_v_target_up =
			  CppAD::pow(current_pixel_v_target_up - constraints.targets_im_up_star.at(j).y, 2);
		  Jim += constraints.weights.w_img_targets.at(j).y * cost_pixel_v_target_up;
		}

		if (constraints.weights.w_img_targets.at(j).z > 0)
		{
		  AD<double> cost_pixel_v_target_down =
			  CppAD::pow(current_pixel_v_target_down - constraints.targets_im_down_star.at(j).y, 2);
		  Jim += constraints.weights.w_img_targets.at(j).z * cost_pixel_v_target_down;
		}

		// Cost_P
		distance_2D_target =
			cinempc::calculateDistanceTo2DPoint<AD<double>>(0, 0, relative_x_target, relative_y_target);
		if (distance_2D_target < minimum_distance_2D_target)
		{
		  minimum_distance_2D_target = distance_2D_target;
		  closest_target_index = j;
		}

		if (constraints.weights.w_d_targets.at(j) > 0)
		{
		  AD<double> cost_d_target = CppAD::pow((distance_2D_target - constraints.targets_d_star.at(j)), 2);
		  Jp += constraints.weights.w_d_targets.at(j) * cost_d_target;
		}

		cinempc::RPY<AD<double>> rot_plot;
		cinempc::RPY<AD<double>> rot_plot_d;
		if (constraints.weights.w_R_targets.at(j) > 0)
		{
		  Eigen::Matrix<AD<double>, 3, 3> drone_R_star =
			  cinempc::quatToRMatrix<AD<double>>(constraints.targets_orientation_star.at(j));
		  Eigen::Matrix<AD<double>, 3, 3> new_drone_R_target = new_drone_R.transpose() * drone_R_target;
		  rot_plot = cinempc::RMatrixtoRPY<AD<double>>(new_drone_R_target);
		  rot_plot_d = cinempc::RMatrixtoRPY<AD<double>>(drone_R_star);
		  if (t == 0)
		  {
			logRPY(drone_R_star, "drone_R_star");
			logRPY(new_drone_R_target, "new_drone_R_target");
		  }

		  AD<double> cost_R_target = (new_drone_R_target - drone_R_star).norm();
		  Jp += constraints.weights.w_R_targets.at(j) * cost_R_target;
		}

		fg[0] += Jp + Jim + JDoF;

		if (t == 0 && j == 0)
		{
		  roll_plot = rot_plot.roll;
		  pitch_plot = rot_plot.pitch;
		  yaw_plot = rot_plot.yaw;

		  roll_plot_d = rot_plot_d.roll;
		  pitch_plot_d = rot_plot_d.pitch;
		  yaw_plot_d = rot_plot_d.yaw;
		  hyper_plot = hyperfocal_distance_mms;
		  dn_plot = near_acceptable_distance;
		  df_plot = far_acceptable_distance;
		  d_target_plot.at(t) = distance_2D_target;
		  current_pixel_target_up_plot.at(t).x = current_pixel_u_target;
		  current_pixel_target_up_plot.at(t).y = current_pixel_v_target_up;
		  current_pixel_target_down_plot.at(t).y = current_pixel_v_target_down;
		  //   d_boy_plot = distance_2D_boy;
		  //   d_girl_plot = distance_2D_girl;
		  //   current_pixel_u_boy_plot = current_pixel_u_boy;
		  //   current_pixel_v_up_boy_plot = current_pixel_v_boy_up;
		  //   current_pixel_v_down_boy_plot = current_pixel_v_boy_down;

		  //   current_pixel_u_girl_plot = current_pixel_u_girl;
		  //   current_pixel_v_up_girl_plot = current_pixel_v_girl_up;
		  //   current_pixel_v_down_girl_plot = current_pixel_v_girl_down;
		  current_Ji = Jim;
		  current_Jp = Jp;
		  current_JDoF = JDoF;

		  std::cout << "Cost:    " << Jp + Jim + JDoF << endl
					<< "   Jp:  " << Jp << endl
					<< "   Jim:  " << Jim << endl
					<< "   JDoF:  " << JDoF << endl
					<< std::endl;

		  std::cout << "DOF " << std::endl
					<< "--------- " << std::endl
					<< "   Dn:  " << near_acceptable_distance << std::endl
					<< "   dn_desired:  " << constraints.dn_star << std::endl
					<< "   Df:  " << far_acceptable_distance << std::endl
					<< "   df_desired:  " << constraints.df_star << std::endl;

		  std::cout << "RELATIVE DISTANCE " << std::endl
					<< "--------- " << std::endl
					<< "   relative_boy_mpc_x_var:  " << relative_x_target << std::endl
					<< "   relative_boy_mpc_y_var:  " << relative_y_target << std::endl
					<< "   relative_boy_mpc_z_var:  " << relative_z_up_target << std::endl
					<< "   relative_boy_mpc_z_down_var:  " << relative_z_down_target << std::endl;

		  std::cout << "IMAGE " << std::endl
					<< "--------- " << std::endl
					<< "   current_pixel_u_boy:  " << current_pixel_u_target << std::endl
					<< "   current_pixel_u_boy_desired:  " << constraints.targets_im_up_star.at(j).x << std::endl
					<< "   current_pixel_v_up_boy:  " << current_pixel_v_target_up << std::endl
					<< "   current_pixel_v_boy_up_desired:  " << constraints.targets_im_up_star.at(j).y << std::endl
					<< "   current_pixel_v_down_boy:  " << current_pixel_v_target_down << std::endl
					<< "   current_pixel_v_boy_down_desired:  " << constraints.targets_im_down_star.at(j).y
					<< std::endl;

		  std::cout << "P " << std::endl
					<< "--------- " << std::endl
					<< "   d_boy:  " << distance_2D_target << std::endl
					<< "   d_boy_desired:  " << constraints.targets_d_star.at(j) << std::endl
					<< "   yaw_boy:  " << std::endl
					<< "   yaw_boy_desired:  " << constraints.targets_orientation_star.at(j).x << endl
					<< "   pitch_boy:  " << std::endl
					<< "   pitch_boy_desired:  " << constraints.targets_orientation_star.at(j).y << std::endl;
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

	  fg[t] = cinempc::calculateDistanceTo3DPoint<AD<double>>(
		  x1, y1, z1, target_states.at(closest_target_index).poses_up.at(t).position.x,
		  target_states.at(closest_target_index).poses_up.at(t).position.y,
		  target_states.at(closest_target_index).poses_up.at(t).position.z);

	  // Setup the rest of the model constraints
	  fg[MPC_N + x_start + t] = x1 - (x0 + vel_x0 * dt);
	  fg[MPC_N + y_start + t] = y1 - (y0 + vel_y0 * dt);
	  fg[MPC_N + z_start + t] = z1 - (z0 + vel_z0 * dt);

	  // rotation dynamics
	  Eigen::Matrix<AD<double>, 3, 3> R = cinempc::RPYtoRMatrix<AD<double>>(roll0, pitch0, yaw0);

	  Eigen::Matrix<AD<double>, 3, 3> R1;
	  Eigen::Matrix<AD<double>, 3, 1> angularVelocityVector(vel_ang_x0 + 0.000001, vel_ang_y0 + 0.000001,
															vel_ang_z0 + 0.000001);

	  Eigen::Matrix<AD<double>, 3, 3> wedgeFromAngularV = expWedgeOperator(angularVelocityVector * dt);

	  R1 = R * wedgeFromAngularV;

	  cinempc::RPY<AD<double>> RPYVector = cinempc::RMatrixtoRPY<AD<double>>(R1);
	  fg[MPC_N + roll_gimbal_start + t] = roll1 - (RPYVector.roll);
	  fg[MPC_N + pitch_gimbal_start + t] = pitch1 - (RPYVector.pitch);
	  fg[MPC_N + yaw_gimbal_start + t] = yaw1 - (RPYVector.yaw);

	  fg[MPC_N + focus_distance_start + t] = focus1 - (focus0 + vel_focus0 * dt);
	  fg[MPC_N + focal_length_start + t] = foc1 - (foc0 + vel_foc0 * dt);
	  fg[MPC_N + aperture_start + t] = aperture1 - (aperture0 + vel_aperture0 * dt);

	  fg[MPC_N + vel_x_start + t] = vel_x1 - (vel_x0 + a_x_0 * dt);
	  fg[MPC_N + vel_y_start + t] = vel_y1 - (vel_y0 + a_y_0 * dt);
	  fg[MPC_N + vel_z_start + t] = vel_z1 - (vel_z0 + a_z_0 * dt);
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
  // N timesteps == N - 1 actuationsb

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

  for (int i = 0; i < target_states.size(); i++)
  {
	Pixel_MPC empty_pixel;
	d_target_plot.push_back(0);
	current_pixel_target_up_plot.push_back(empty_pixel);
	current_pixel_target_down_plot.push_back(empty_pixel);
  }

  // Set the initial variable values
  vars[x_start] = msg->drone_state.drone_pose.position.x;
  vars[y_start] = msg->drone_state.drone_pose.position.y;
  vars[z_start] = msg->drone_state.drone_pose.position.z;

  cinempc::RPY<double> rpy = cinempc::quatToRPY<double>(msg->drone_state.drone_pose.orientation);

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
  // to the max negative and positive values.

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.

  for (int i = 0; i < n_vars; i++)
  {
	double lowerbound, upperbound;
	if (i >= x_start && i < y_start)
	{
	  lowerbound = x_lowest;
	  upperbound = x_highest;
	}
	else if (i >= y_start && i < z_start)
	{
	  lowerbound = y_lowest;
	  upperbound = y_highest;
	}
	else if (i >= z_start && i < roll_gimbal_start)
	{
	  lowerbound = -0.5;  // y_lowest;
	  upperbound = msg->floor_pos - 0.8;
	}
	else if (i >= roll_gimbal_start && i < pitch_gimbal_start)
	{
	  lowerbound = -0.5;
	  upperbound = 0.5;
	}
	else if (i >= pitch_gimbal_start && i < yaw_gimbal_start)
	{
	  lowerbound = -0.5;
	  upperbound = 0.5;
	}
	else if (i >= yaw_gimbal_start && i < focus_distance_start)
	{
	  lowerbound = -0.5;
	  upperbound = 0.5;
	}
	else if (i >= focus_distance_start && i < focal_length_start)
	{
	  if (use_cineMPC)
	  {
		lowerbound = 4;
		upperbound = 70;
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
		lowerbound = 30;
		upperbound = 300;
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
		lowerbound = 1;
		upperbound = 25;
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
	  lowerbound = -15;
	  upperbound = 15;
	}
	else if (i >= vel_focal_length_start && i < vel_aperture_start)
	{
	  if (use_cineMPC)
	  {
		lowerbound = -15;
		upperbound = 15;
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
		lowerbound = -15;
		upperbound = 15;
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

  // *** Lower and upper limits for the constraints
  // Should be 0 besides initial state.
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
	constraints_lowerbound[i] = 3;
	constraints_upperbound[i] = 1000;
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

	response_msg.plot_values.push_back(Value(hyper_plot));
	// response_msg.plot_values.push_back(Value(vel_ang_x));
	// response_msg.plot_values.push_back(Value(vel_ang_y));
	// response_msg.plot_values.push_back(Value(vel_ang_z));
	response_msg.plot_values.push_back(Value(roll_plot));
	response_msg.plot_values.push_back(Value(pitch_plot));
	response_msg.plot_values.push_back(Value(yaw_plot));
	response_msg.plot_values.push_back(Value(roll_plot_d));
	response_msg.plot_values.push_back(Value(pitch_plot_d));
	response_msg.plot_values.push_back(Value(yaw_plot_d));
	response_msg.plot_values.push_back(Value(dn_plot));
	response_msg.plot_values.push_back(Value(df_plot));
	for (int i = 0; i < target_states.size(); i++)
	{
	  response_msg.plot_values.push_back(Value(d_target_plot.at(i)));
	  response_msg.plot_values.push_back(Value(current_pixel_target_up_plot.at(i).x));
	  response_msg.plot_values.push_back(Value(current_pixel_target_up_plot.at(i).y));
	  response_msg.plot_values.push_back(Value(current_pixel_target_down_plot.at(i).x));
	}
	response_msg.plot_values.push_back(Value(current_Ji));
	response_msg.plot_values.push_back(Value(current_Jp));
	response_msg.plot_values.push_back(Value(current_JDoF));

	results_pub.publish(response_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cinempc_solver");
  ros::NodeHandle n;

  ros::ServiceClient service_take_off = n.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
  airsim_ros_pkgs::Takeoff srv;
  srv.request.waitOnLastTask = false;

  service_take_off.call(srv);

  ros::Subscriber new_state_received_sub =
	  n.subscribe<cinempc::MPCIncomingState>("cinempc/current_state", 1000, newStateReceivedCallback);
  results_pub = n.advertise<cinempc::MPCResult>("cinempc/next_n_states", 100);
  ros::spin();
  return 0;
}