#include <string>
#include <vector>

const double PI = 3.14159265358979323846;  //(mm)

// position of the subject to record
// rotations

const double target_height = 1.62, target_width = 0.5;

const bool static_target = true, use_perception = true, drone_moving = true, use_cineMPC = true, log_costs = true,
           dynamic_scene = true;

const double target_yaw_gt = PI / 2, target_pitch_gt = 0, drone_start_yaw = -PI / 2;

// constant values. Size of the sensor in mm and px
const double sensor_height_mm = 13.365, sensor_width_mm = 23.76;  //(mm) Filmback, Digital Film 16:9
const double picture_height_px = 540, picture_width_px = 960;     //(px)
const double sim_speed = 1;
const double sim_frequency = 1 / sim_speed;

std::vector<std::string> targets_names = { "Tuareg" };
std::vector<std::string> targets_classes = { "person" };

const double image_x_third_left = picture_width_px / 3;
const double image_x_third_right = 2 * picture_width_px / 3;
const double image_x_center = picture_width_px / 2;

const double image_y_third_down = 2 * picture_height_px / 3;
const double image_y_third_up = picture_height_px / 3;
const double image_y_center = picture_height_px / 2;

const int MPC_N = 5;
const float mpc_dt = 0.4 * sim_speed;  // sample period MPC

const double min_value = -99999999, max_value = 99999999;

// constraints
const double x_lowest = min_value, x_highest = x_highest;
const double y_lowest = min_value, y_highest = max_value;
const double z_lowest = -0.5, z_highest = 50;  // z_higuest depends on floor

const double roll_lowest = -0.03, roll_highest = 0.03;
const double pitch_lowest = -0.03, pitch_highest = 0.03;
const double yaw_lowest = -0.03, yaw_highest = 0.03;

const double foc_lowest = 30, foc_highest = 500;     // 30 500
const double focus_lowest = 4, focus_highest = 100;  // 4 70
const double aperture_lowest = 1, aperture_highest = 22;

const double velos = 2 * sim_speed;
const double vel_x_lowest = -velos, vel_x_highest = velos;
const double vel_y_lowest = -velos, vel_y_highest = velos;
const double vel_z_lowest = -velos, vel_z_highest = velos;

const double vel_ang = 0.03 * sim_speed;
const double vel_ang_x_lowest = -vel_ang, vel_ang_x_highest = vel_ang;
const double vel_ang_y_lowest = -vel_ang, vel_ang_y_highest = vel_ang;
const double vel_ang_z_lowest = -vel_ang, vel_ang_z_highest = vel_ang;

const double vel_foc_lowest = -20 * sim_speed, vel_foc_highest = 20 * sim_speed;  // -15 25
const double vel_focus_lowest = -15 * sim_speed, vel_focus_highest = 15 * sim_speed;
const double vel_aperture_lowest = -3 * sim_speed, vel_aperture_highest = 3 * sim_speed;

const double accel = 1.2 * sim_speed;
const double a_x_lowest = -accel, a_x_highest = accel;
const double a_y_lowest = -accel, a_y_highest = accel;
const double a_z_lowest = -accel, a_z_highest = accel;

const double circle_confusion = 0.05;  //(mm)

const double tolerance_pixel_x = 7;
const double tolerance_pixel_y = 5;
const double tolerance_reduce_weight = 0.01;

// Definition of sequences
const int start_sequence_1 = 0;  //
const int start_sequence_2_5 = 20;
const int start_sequence_3 = 27;

const std::string project_folder = "/home/pablo/Desktop/AirSim_update/AirSim_ros/ros/src/cinempc/";

const std::string config_file_yolo = "/home/pablo/Downloads/darknet/cfg/yolov4.cfg";
const std::string weights_file_yolo = "/home/pablo/Downloads/darknet/yolov4.weights";
const std::string names_file_yolo = "/home/pablo/Downloads/darknet/cfg/coco.names";

const int kf_states = 6, kf_measurements = 3;
const double kf_dt = 0.2;
const double kf_a = 1, kf_c = 1, kf_q = 0.5, kf_r = 0.5, kf_p = 2;
const double kf_init_x = 0, kf_init_y = -10, kf_init_z = 0, kf_init_vx = 0, kf_init_vy = 0, kf_init_vz = 0;