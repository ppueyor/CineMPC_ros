#include <string>
#include <vector>

const double PI = 3.14159265358979323846;  //(mm)

const bool static_target = false, use_perception = false, drone_moving = false, use_cineMPC = false, log_costs = false,
           dynamic_scene = false, publish_topics_gt = false, floor_constraints = false, take_off_when_start = false,
           save_imgs = false;

// simulation and MPC variables
const double sim_speed = 1;
const double sim_frequency = 1 / sim_speed;
double steps_each_dt_low_level = 0;
const int MPC_N = 5;
const float mpc_dt = 0.2 * sim_speed;  // sample period MPC

// Camera and drone parameters
const double drone_start_yaw = 0;
const double sensor_height_mm = 0, sensor_width_mm = 0;    //(mm) Filmback, Digital Film 16:9
const double picture_height_px = 0, picture_width_px = 0;  //(px)
const double circle_confusion = 0;                         //(mm)

const double image_x_third_left = picture_width_px / 3;
const double image_x_third_right = 2 * picture_width_px / 3;
const double image_x_center = picture_width_px / 2;

const double image_y_third_down = 2 * picture_height_px / 3;
const double image_y_third_up = picture_height_px / 3;
const double image_y_center = picture_height_px / 2;

// Targets parameters
std::vector<std::string> targets_names = { "Target" };
std::vector<std::string> targets_classes = { "target" };
const bool is_person = false;
const double target_1_yaw_gt = 0, target_1_pitch_gt = 0;
const double target_1_height = 0, target_1_width = 0;

// C-Set of constraints of the problem
const double x_lowest = -0, x_highest = 0;
const double y_lowest = -0, y_highest = 0;
const double z_lowest = -0, z_highest = 0;

const double roll_lowest = -0, roll_highest = 0;
const double pitch_lowest = -0, pitch_highest = 0;
const double yaw_lowest = -0, yaw_highest = 0;

const double foc_lowest = 0, foc_highest = 0;
const double focus_lowest = 0, focus_highest = 0;
const double aperture_lowest = 0, aperture_highest = 0;

const double constraint_velocity = 0 * sim_speed;
const double vel_x_lowest = -constraint_velocity, vel_x_highest = constraint_velocity;
const double vel_y_lowest = -constraint_velocity, vel_y_highest = constraint_velocity;
const double vel_z_lowest = -constraint_velocity, vel_z_highest = constraint_velocity;

const double constraint_accel = 0 * sim_speed;
const double a_x_lowest = -constraint_accel, a_x_highest = constraint_accel;
const double a_y_lowest = -constraint_accel, a_y_highest = constraint_accel;
const double a_z_lowest = -constraint_accel, a_z_highest = constraint_accel;

const double constraint_vel_ang = 0 * sim_speed;
const double vel_ang_x_lowest = -constraint_vel_ang, vel_ang_x_highest = constraint_vel_ang;
const double vel_ang_y_lowest = -constraint_vel_ang, vel_ang_y_highest = constraint_vel_ang;
const double vel_ang_z_lowest = -constraint_vel_ang, vel_ang_z_highest = constraint_vel_ang;

const double vel_foc_lowest = -0 * sim_speed, vel_foc_highest = 0 * sim_speed;
const double vel_focus_lowest = -0 * sim_speed, vel_focus_highest = 0 * sim_speed;
const double vel_aperture_lowest = -0 * sim_speed, vel_aperture_highest = 0 * sim_speed;

// tolerances for Jim
const double tolerance_pixel_x = 0;
const double tolerance_pixel_y = 0;
const double tolerance_reduce_weight = 0;

// Definition of sequences
const int start_sequence_x = 0;
const int start_sequence_x2 = 0;

// Definition of folders
const std::string project_folder = "/project/folder/";
const std::string config_file_yolo = "/yolo/config/yolov.cfg";
const std::string weights_file_yolo = "/yolo/config/yolov.weights";
const std::string names_file_yolo = "/yolo/config/coco.names";

// KF_parameters
const int kf_states = 0, kf_measurements = 0;
const double kf_dt = 0;
const double kf_a = 0, kf_c = 0, kf_q = 0, kf_r = 0, kf_p = 0;
const double kf_init_x = 0, kf_init_y = 0, kf_init_z = 0, kf_init_vx = 0, kf_init_vy = 0, kf_init_vz = 0;