#ifndef MPC_H
#define MPC_H

#include <airsim_ros_pkgs/DroneAndCameraState.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <cinempc/MPCIncomingState.h>
#include <cinempc/MPCResult.h>
#include <cinempc/TargetStates.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <vector>

#include "Constants.h"
#include "common/common_utils/Utils.hpp"
#include "ros/ros.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

using namespace std;

struct RPY_AD
{
  CppAD::AD<double> roll;
  CppAD::AD<double> pitch;
  CppAD::AD<double> yaw;
  RPY_AD(){};
  RPY_AD(CppAD::AD<double> rollc, CppAD::AD<double> pitchc, CppAD::AD<double> yawc)
    : roll(rollc), pitch(pitchc), yaw(yawc)
  {
  }
};
// too
class MPC
{
public:
  MPC();

  virtual ~MPC();
};

#endif /* MPC_H */
