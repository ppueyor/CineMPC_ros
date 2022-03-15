
[Time, Sequence, Sim_time, mpc_dt, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d,relative_roll, relative_pitch, relative_yaw, relative_roll_d, relative_pitch_d, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'Sim_time', 'mpc_dt', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_roll', 'relative_pitch', 'relative_yaw', 'relative_roll_d', 'relative_pitch_d', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});


close all
window = 5;
Time = Time / 10^3;
idx = find( Time >= 0 & Time < 500);

currDate = strrep(datestr(datetime), ':', '_');
dir = mkdir('Results',currDate)
path = strcat('Results/',currDate)

copyfile('log_file.csv',path);


figy=figure;
plot2=plot(Time(idx), focal_length(idx), 'LineWidth',3);
hold on 
legend('focal\_length', 'focal\_length\_d')
saveas(figy,fullfile(path,'focal_l.jpg'));


figdof=figure;
plot3=plot(Time(idx), relative_pitch(idx), 'LineWidth',3);
hold on 
plot(Time(idx), relative_pitch_d(idx), 'LineWidth',3)
plot(Time(idx), relative_yaw(idx), 'LineWidth',3)
plot(Time(idx), relative_yaw_d(idx), 'LineWidth',3)
plot(Time(idx), relative_roll(idx), 'LineWidth',3)
plot(Time(idx), relative_roll_d(idx), 'LineWidth',3)

legend('relative\_pitch','relative\_pitch\_d','relative\_yaw','relative\_yaw\_d')
saveas(figdof,fullfile(path,'rot_cost.jpg'));


figd=figure;
plot3=plot(Time(idx), d_gt(idx), 'LineWidth',3);
hold on 
plot(Time(idx), d_d(idx), 'LineWidth',3)
legend('d','d\_d')
saveas(figd,fullfile(path,'d.jpg'));

figim=figure;
plot3=plot(Time(idx), im_u(idx), 'LineWidth',3);
hold on 
plot(Time(idx), im_u_d(idx), 'LineWidth',3)
plot(Time(idx), im_v_up(idx), 'LineWidth',3)
plot(Time(idx), im_v_up_d(idx), 'LineWidth',3)
plot(Time(idx), im_v_down(idx), 'LineWidth',3)
plot(Time(idx), im_v_down_d(idx), 'LineWidth',3)
legend('im\_u','im\_u\_d','im\_v\_up', 'im\_v\_up\_d','im\_down\_up', 'im\_v\_down\_d')
saveas(figim,fullfile(path,'im.jpg'));


figpos = figure;
plot4=plot(Time(idx), world_x_gt(idx), 'LineWidth',3);
hold on
plot4=plot(Time(idx), world_y_gt(idx), 'LineWidth',3);
plot4=plot(Time(idx), world_z_gt(idx), 'LineWidth',3);

plot4=plot(Time(idx), world_x_perception(idx), 'LineWidth',3);
plot4=plot(Time(idx), world_y_perception(idx), 'LineWidth',3);
plot4=plot(Time(idx), world_z_perception(idx), 'LineWidth',3);

plot4=plot(Time(idx), world_x_kf(idx), 'LineWidth',3);
plot4=plot(Time(idx), world_y_kf(idx), 'LineWidth',3);
plot4=plot(Time(idx), world_z_kf(idx), 'LineWidth',3);
legend('x\_gt','y\_gt','z\_gt', 'x\_measure', 'y\_measure', 'z\_measure', 'x\_kf', 'y\_kf', 'z\_kf')
saveas(figpos,fullfile(path,'pos.jpg'));


figrot = figure;
plot4=plot(Time(idx), yaw_gt(idx) + 1.57, 'LineWidth',3);
hold on
plot4=plot(Time(idx), pitch_gt(idx), 'LineWidth',3);
plot4=plot(Time(idx), yaw_perception(idx), 'LineWidth',3);

plot4=plot(Time(idx), pitch_perception(idx), 'LineWidth',3);

legend('yaw\_gt','pitch\_gt','yaw\_perception','pitch\_perception')
saveas(figrot,fullfile(path,'rot.jpg'));

