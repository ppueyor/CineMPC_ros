cmake_minimum_required(VERSION 3.10.0)
project(cinempc)

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 


find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation 
  nav_msgs
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
  IntrinsicsCamera.msg
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
  ${RPC_LIB_INCLUDES}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(cinempc_scene_node src/user/cinempc_scene_node.cpp)
add_dependencies(cinempc_scene_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(cinempc_scene_node airsim_ros   ${catkin_LIBRARIES})

add_executable(cinempc_change_sequence_node src/user/cinempc_change_sequence_node.cpp)
add_dependencies(cinempc_change_sequence_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(cinempc_change_sequence_node ${catkin_LIBRARIES})

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

add_executable(cinempc_perception_estimation_node src/cinempc/cinempc_perception_estimation_node.cpp src/cinempc/cinempc_kalman.cpp) 
add_dependencies(cinempc_perception_estimation_node cinempc_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(cinempc_perception_estimation_node ${catkin_LIBRARIES} )
target_link_libraries(cinempc_perception_estimation_node ${catkin_LIBRARIES} )


install(TARGETS
    #list of nodes
    cinempc_scene_node
    cinempc_mpc_node
    cinempc_low_level_node
    cinempc_perception_measurement_node
    cinempc_perception_estimation_node
    cinempc_main_node
    cinempc_change_sequence_node
    
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
