[Time, Sequence, drone_x, drone_y, drone_z, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d, relative_yaw, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence',  'drone_x', 'drone_y', 'drone_z', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_yaw', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});

idx = find( Time > 0& Time < 80000 );
figure;
f = focal_length(idx);
df = abs(diff(f));
df=[df;0]/12


a = aperture(idx);
da = abs(diff(a));
da=[da;0]/1.18

d_x = drone_x(idx);
dx = abs(diff(d_x));
dx = [dx;0];

d_y = drone_y(idx);
dy = abs(diff(d_y));
dy = [dy;0];

d_z = drone_z(idx);
dz = abs(diff(d_z));
dz = [dz;0];


% plot(Time(idx) / 1000, (focal_length(idx)-20)/180,'LineWidth',2);

%ylim([0 70000]);

% vel_ang_x_norm = (vel_ang_x(idx)+0.5)/1;
% vel_ang_y_norm = (vel_ang_y(idx)+0.5)/1;
% vel_ang_z_norm = (vel_ang_z(idx)+0.5)/1;


plot1 = plot(Time(idx) / 1000, dx,'o-', 'LineWidth',8, 'Markersize',20, 'MarkerIndices',1:5:idx(end));
hold on
plot2 = plot(Time(idx) / 1000, dy,'o-', 'LineWidth',8, 'Markersize',20, 'MarkerIndices',1:5:idx(end));
plot2 = plot(Time(idx) / 1000, dz,'o-', 'LineWidth',8, 'Markersize',20, 'MarkerIndices',1:5:idx(end));
plot1 = plot(Time(idx) / 1000, df,'mo-', 'LineWidth',8, 'Markersize',20, 'MarkerIndices',1:5:idx(end));
hold on
plot2 = plot(Time(idx) / 1000, da,'bo-', 'LineWidth',8, 'Markersize',20, 'MarkerIndices',1:5:idx(end));


%plot(Time(idx) / 1000, aperture(idx),'LineWidth',2);

%plot(Time(idx) / 1000, 10*seq(idx),'LineWidth',2);

%plot(Time(idx) / 1000, focus_distance(idx),'LineWidth',2);

ylim([0 1]);
ax = gca;
ax.FontSize = 70; 
% legend('f_k', '||v||', '||\Omega||')
lgd = legend([plot1 plot2 plot3],{'|v_f|','||a_d||','||\Omega_d||'}, 'Orientation', 'Horizontal')
lgd.FontSize = 100;
%plot(Time / 1000, Sequence * 10000,'LineWidth',2);