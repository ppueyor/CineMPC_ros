cmake_minimum_required(VERSION 3.10.0)
project(cinempc)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../)

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
# find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation 
  nav_msgs
  # nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geographic_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

add_message_files(
  FILES
  MPCIncomingState.msg
  DroneAndCameraState.msg
  WeightsTargets.msg
  MeasurementOut.msg  
  MeasurementIn.msg  
  TargetState.msg
  MPCResult.msg
  EstimationIn.msg
  MPCResultPlotValues.msg
  PlotValues.msg
  LowLevelControl.msg
  TargetStateMPC.msg
  Constraints.msg
  Weights.msg
)

add_service_files(
  FILES
  GetNNextTargetPoses.srv
  GetUserConstraints.srv
) 

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
  sensor_msgs
  geographic_msgs
  airsim_ros_pkgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs airsim_ros_pkgs # nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(scene_node src/user/cinempc_scene_node.cpp)
add_dependencies(scene_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(scene_node airsim_ros   ${catkin_LIBRARIES})

add_executable(change_sequence_node src/user/change_sequence_node.cpp)
add_dependencies(change_sequence_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(change_sequence_node ${catkin_LIBRARIES})

add_executable(cinempc_user_node src/user/cinempc_user_node.cpp) 
add_dependencies(cinempc_user_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(cinempc_user_node ${catkin_LIBRARIES} )

add_executable(cinempc_mpc_node src/cinempc/cinempc_mpc_node.cpp) 
add_dependencies(cinempc_mpc_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(cinempc_mpc_node ipopt ${catkin_LIBRARIES} )

add_executable(cinempc_main_node src/cinempc/cinempc_main_node.cpp) 
add_dependencies(cinempc_main_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(cinempc_main_node ${catkin_LIBRARIES} )

add_executable(cinempc_low_level_node src/cinempc/cinempc_low_level_node.cpp) 
add_dependencies(cinempc_low_level_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(cinempc_low_level_node  ${catkin_LIBRARIES} )

add_executable(cinempc_perception_measurement_node src/cinempc/cinempc_perception_measurement_node.cpp) 
add_dependencies(cinempc_perception_measurement_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(cinempc_perception_measurement_node darkhelp darknet ${catkin_LIBRARIES} AirLib)

add_executable(cinempc_perception_estimation_node src/cinempc/cinempc_perception_estimation_node.cpp src/cinempc/kalman.cpp) 
add_dependencies(cinempc_perception_estimation_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(cinempc_perception_estimation_node ${catkin_LIBRARIES} )


install(TARGETS
    #list of nodes
    scene_node
    cinempc_mpc_node
    cinempc_low_level_node
    cinempc_perception_measurement_node
    cinempc_perception_estimation_node
    cinempc_main_node
    change_sequence_node
    
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    #list of shared libraries
    
    #pd_position_controller_simple
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  # settings.json
  # nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
