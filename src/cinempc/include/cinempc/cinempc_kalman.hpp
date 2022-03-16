/**
 * Kalman filter implementation using Eigen. Based on the following
 * introductory paper:
 *
 *     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <stdio.h>

#include <eigen3/Eigen/Dense>
#pragma once

class KalmanFilterEigen
{
public:
  /**
   * Create a Kalman filter with the specified matrices.
   *   A - System dynamics matrix
   *   C - Output matrix
   *   Q - Process noise covariance
   *   R - Measurement noise covariance
   *   P - Estimate error covariance
   */
  KalmanFilterEigen(double dt, const Eigen::MatrixXd& A, const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R, const Eigen::MatrixXd& P, int n, int m);

  /**
   * Create a blank estimator.
   */
  KalmanFilterEigen();

  /**
   * Initialize the filter with initial states as zero.
   */
  void init();

  /**
   * Initialize the filter with a guess for initial states.
   */
  void init(double t0, const Eigen::VectorXd& x0);

  int numberOfMeasurements();

  int numberOfStates();
  /**
   * Update the estimated state based on measured values. The
   * time step is assumed to remain constant.
   */
  void update(const Eigen::VectorXd& y, bool measure);

  Eigen::MatrixXd predict(int kf_time_each_mpc, int max_time_steps);

  /**
   * Update the estimated state based on measured values,
   * using the given time step and dynamics matrix.
   */
  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

  /**
   * Update the estimated state based on measured values, without measure
   * using the given time step and dynamics matrix.
   */
  void update(double dt, const Eigen::MatrixXd A);

  /**
   * Return the current state and time.
   */
  Eigen::VectorXd state()
  {
    return x_hat;
  };
  double time()
  {
    return t;
  };

  double get_dt()
  {
    return dt;
  };

private:
  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};