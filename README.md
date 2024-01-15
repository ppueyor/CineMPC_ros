# Examples with real drones and in simulation

[![YouTube](https://img.youtube.com/vi/lgQPEApnQIE/0.jpg)](https://www.youtube.com/watch?v=lgQPEApnQIE)

# CineMPC++ installation guide
  
CineMPC++ is ready to play on a drone platform using ROS. The infrastructure to test it in the photorealistic simulator AirSim is provided as an example of use.  

## Prerequisites  
The prerequisites that must be installed before running the experiments are:
- git (`sudo apt install git`)  
- Unreal Engine and AirSim -> https://microsoft.github.io/AirSim/build_linux/ (if using AirSim)
- OpenCV ([https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html))    
- Darknet, DarkHelp ([https://www.ccoderun.ca/DarkHelp/api/Building.html](https://www.ccoderun.ca/DarkHelp/api/Building.html))  
- ROS ([http://wiki.ros.org/es/ROS/Installation](http://wiki.ros.org/es/ROS/Installation))  
- mav_ros (`sudo apt-get install ros-'ros_distribution'-mavros`)  
- tf2-sensor-msgs (`sudo apt-get install ros-'ros_distribution'-tf2-sensor-msgs`)  
- CppAD (`sudo apt-get install cppad`)  
- IpOpt ([https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md))  
- cv_bridge: Copy the packages in "[https://github.com/ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv)" in the catkin_ws and compile them.  
  
  
## Things to be configured in advance  
  
1. `cinempc/include/Constants.h` ->  
  - `project_folder` -> Project folder where the project is saved  
- `config_file_yolo` -> Path of the .cfg file of Yolo (found in the installation folder of Darknet/cfg - yolov4.cfg )  
- `weights_file_yolo` -> Path of the .weights file of Yolo ([https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights))  
- `names_file_yolo` -> Path of the .names file of Yolo (found in the installation folder of Darknet/cfg - coco.names )
2. `cinempc/include/Constants.h` (line 113 onwards). Replace the topics and services of ROS with the ones used by the platform. The topics now are ready to use AirSim.
3. Add the targets of the experiment in the file `cinempc/include/Constants.h`:
    - `targets_names` -> Add or replace the names of the recorded targets
- `targets_classes` -> Add or replace the classes of the recorded targets
4. Compile the project `(catkin_make)` from the root folder.  
5. `source devel/setup.bash`; (or add to `source devel/setup.bash >> ~/.bashrc`)  


  
## Configuring CineMPC++ and AirSim 
The experiments communicate with AirSim using ROS. An edition of the AirSim ROS wrapper (https://microsoft.github.io/AirSim/airsim_ros_pkgs/) is added to the repository. It includes some new topics/services to change the scene and the state of the drone and the camera intrinsics, according to the user's requirements. 

1. `src/sirsim_ros_pkgs/CMakeLists.txt` -> AIRSIM_ROOT (line 5) to the path of the AirSim installation root folder

2. The specifications that the user should introduce are included in the files under the folder `/cinempc/src/user` 
- `cinempc_user_node.cpp` -> weights and setpoints for every cost term 
- `cinempc_scene_node.cpp ->`behaviour of the elements of the scene-  
- `cinempc_change_sequence_node.cpp ->`sequences of the experiment. The file `cinempc/include/Constants.h` contains an example of sequences. (lines 84-91)
  
3. The requirements of the experiments (information about targets, constraints, starting points, debugging options...) can be edited in the file `Constants.h` inside the folder `/cinempc/include/user`.
  
There is a generic version of these files in `cinempc/src/user `so the values can be edited directly. 
To reproduce the two experiments of the manuscript, rename the folder to `user` of the experiments to run:
-	`/cinempc/src/user_plane, /cinempc/include/user_plane` to `/cinempc/src/user, /cinempc/include/user`
-	`/cinempc/src/user_dolly, /cinempc/include/user_dolly`  to  `/cinempc/src/user, /cinempc/include/user`
-	 Copy the settings file under `AirSim_settings` to the appropriate directory
4. Add the targets of the experiment in the file `airsim_ros_pkgs/launch/airsim_node.launch` into the field "targets". Example: `<arg name="targets" default="[Plane"] />`

	
## Running the example with AirSim
  
1. Open your project in AirSim and run it.
3. Launch `roslaunch airsim_ros_pkgs airsim_all.launch` in one terminal
4. Launch `roslaunch cinempc cinempc_all.launch` In another terminal
5. Let the magic start!

# Paper available

https://ieeexplore.ieee.org/document/10398502


# Cite us!

```
@ARTICLE{10398502,
  author={Pueyo, Pablo and Dendarieta, Juan and Montijano, Eduardo and Murillo, Ana C. and Schwager, Mac},
  journal={IEEE Transactions on Robotics}, 
  title={CineMPC: A Fully Autonomous Drone Cinematography System Incorporating Zoom, Focus, Pose, and Scene Composition}, 
  year={2024},
  volume={},
  number={},
  pages={1-18},
  doi={10.1109/TRO.2024.3353550}}
```
