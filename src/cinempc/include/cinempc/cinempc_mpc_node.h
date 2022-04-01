#ifndef MPC_H
#define MPC_H

#include <airsim_ros_pkgs/Takeoff.h>
#include <cinempc/CineMPCCommon.h>
#include <cinempc/MPCIncomingState.h>
#include <cinempc/MPCResult.h>
#include <cinempc/MPCResultPlotValues.h>
#include <cinempc/TargetStateMPC.h>
#include <std_msgs/Float32.h>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <vector>

#include "cinempc/cinempc_mpc_node.h"
#include "common/common_utils/Utils.hpp"
#include "cppad/ipopt/solve.hpp"
#include "ros/ros.h"

using namespace std;
using CppAD::AD;
using namespace Eigen;
class MPC
{
public:
  MPC();

  virtual ~MPC();
};

#endif /* MPC_H */
