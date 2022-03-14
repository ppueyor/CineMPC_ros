#ifndef MPC_H
#define MPC_H

#include <CineMPCCommon.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <cinempc/DroneAndCameraState.h>
#include <cinempc/MPCIncomingState.h>
#include <cinempc/MPCResult.h>
#include <cinempc/MPCResultPlotValues.h>
#include <cinempc/TargetStateMPC.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <vector>

#include "common/common_utils/Utils.hpp"
#include "ros/ros.h"

using namespace std;

// too
class MPC
{
public:
  MPC();

  virtual ~MPC();
};

#endif /* MPC_H */
