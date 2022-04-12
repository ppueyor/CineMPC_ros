
# CineMPC++ installation guide!  
  
CineMPC++ is ready to play in a drone platform using ROS. The infraestucture to test it in the photorealistic simulator AirSim is provided.  
  
## How it works  
The experiments communicate with AirSim using ROS. An edition of the AirSim ROS wrapper (https://microsoft.github.io/AirSim/airsim_ros_pkgs/) is added to the repository. It includes some new topics/services to change the scene and the state of the drone and the camera intrinsics, according to the user's requirements. 
  
The specifications that the user should introduce are included in the files under the folder `/cinempc/src/user` 
- `cinempc_user_node.cpp` -> weights and setpoints for every cost term 
- `cinempc_scene_node.cpp ->`behaviour of the elements of the scene-  
- `cinempc_change_sequence_node.cpp ->`sequences of the experiment
  
The requirements of the experiments (information about targets, constraints, starting points, debugging options...) can be edit in the file `Constants.h` inside the folder `/cinempc/include/user`.
  
There is a generic version of these files in `cinempc/src/user `so the values can be edited directly. 
To reproduce the two experiments of the manuscript, rename the folder to `user` of the experiments to run:
-	`/cinempc/src/user_plane, /cinempc/include/user_plane` to `/cinempc/src/user, /cinempc/include/user`
-	`/cinempc/src/user_dolly, /cinempc/include/user_dolly`  to  `/cinempc/src/user, /cinempc/include/user`
-	Copy the settings file under `AirSim_settings` to the appropiate directory
  
## Prerequisites  
The prerrequisites that must be installed before running the experiments are:
- git (`sudo apt install git`)  
- Unreal Engine and AirSim -> https://microsoft.github.io/AirSim/build_linux/ 
- OpenCV ([https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html))    
- Darknet, DarkHelp ([https://www.ccoderun.ca/DarkHelp/api/Building.html](https://www.ccoderun.ca/DarkHelp/api/Building.html))  
- ROS ([http://wiki.ros.org/es/ROS/Installation](http://wiki.ros.org/es/ROS/Installation))  
- mav_ros (`sudo apt-get install ros-'ros_distribution'-mavros`)  
- tf2-sensor-msgs (`sudo apt-get install ros-'ros_distribution'-tf2-sensor-msgs`)  
- CppAD (`sudo apt-get install cppad`)  
- IpOpt ([https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md))  
- cv_bridge: Copy the packages in "[https://github.com/ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv)" in the catkin_ws and compile them.  
  
  
## Things to be configured in advance  
  
1. `src/cinempc/CMakeLists.txt` -> AIRSIM_ROOT (line 5) to the path of the AirSim installation root folder  
2. `src/sirsim_ros_pkgs/CMakeLists.txt` -> AIRSIM_ROOT (line 5) to the path of the AirSim installation root folder  
3. `cinempc/include/Constants.h` ->  
  - `project_folder` -> Project folder where the project is saved  
- `config_file_yolo` -> Path of the .cfg file of Yolo (found in the installation folder of Darknet/cfg - yolov4.cfg )  
- `weights_file_yolo` -> Path of the .weights file of Yolo ([https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights))  
- `names_file_yolo` -> Path of the .names file of Yolo (found in the installation folder of Darknet/cfg - coco.names )  
4. Compile the project `(catkin_make)` from the root folder.  
5. `source devel/setup.bash`; (or add to `source devel/setup.bash >> ~/.bashrc`)  
6. Add the targets of the experiment in the file `airsim_ros_pkgs/launch/airsim_node.launch` into the field "targets". Example: `<arg name="targets" default="[Plane"] />` 
## Running an example  
  
1. Open your project in AirSim and run it.
3. Launch `roslaunch airsim_ros_pkgs airsim_all.launch` in one terminal
4. Launch `roslaunch cinempc cinempc_all.launch` In another terminal
5. Let the magic start!
