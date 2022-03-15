
[Time, Sequence, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d, relative_yaw, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_yaw', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});


close all
window = 5;
Time = Time / 10^3;
idx = find( Time >= 0 & Time < 500);
id_seq2 = find( Sequence == 2 & Time > 20);
id_seq1 = find( Sequence == 0);

d_person = dn_d -0.5;
d_cactus_far = d_person +46.4; 
d_cactus_close = d_person - 22.3; 


currDate = strrep(datestr(datetime), ':', '_');
dir = mkdir('Results',currDate)
path = strcat('Results/',currDate)

copyfile('log_file.csv',path);


%lg = legend('x', 'y', 'z')
figy=figure;
plot2=plot(Time(idx), focal_length(idx), 'LineWidth',3);
hold on 
plot(Time(idx), focal_length_d(idx), 'LineWidth',3)
plot3=plot(Time(idx), d_gt(idx) * 10, 'LineWidth',3);
legend('focal\_length', 'focal\_length\_d', 'dx10')
saveas(figy,fullfile(path,'focal_l.jpg'));

%lg = legend('x', 'y', 'z')
figy=figure;
plot2=plot(Time(idx), aperture(idx), 'LineWidth',3);
legend('aperture')
saveas(figy,fullfile(path,'aperture.jpg'));

%lg = legend('x', 'y', 'z')
figy=figure;
plot2=plot(Time(idx), focus_distance(idx)/100, 'LineWidth',3);
legend('focus\_distance')
ylim([0 60])
saveas(figy,fullfile(path,'focus_distance.jpg'));




figdof=figure;
plot3=plot(Time(idx), dn(idx), 'LineWidth',3);
hold on 
plot(Time(idx), dn_d(idx), 'LineWidth',3)
plot(Time(id_seq2), df(id_seq2), 'LineWidth',3)
plot(Time(id_seq2), df_d(id_seq2), 'LineWidth',3)
plot(Time(idx), d_cactus_far(idx), 'LineWidth',3)
plot(Time(idx), d_cactus_close(idx), 'LineWidth',3)

legend('dn','dn\_d','df','df\_d', 'd_cactus_close', 'd_cactus_far')
saveas(figdof,fullfile(path,'dof.jpg'));


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
plot(Time(idx), im_v_center(idx), 'LineWidth',3)
plot(Time(idx), im_v_center_d(idx), 'LineWidth',3)
legend('im\_u','im\_u\_d','im\_v\_up', 'im\_v\_up\_d')
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


