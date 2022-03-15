
[Time, Sequence, drone_x, drone_y, drone_z, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d, relative_yaw, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence',  'drone_x', 'drone_y', 'drone_z', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_yaw', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});
close all
figure('units','normalized','outerposition',[0.25 0 0.75 1]);
%plot(960-u_boy(idx),Time(idx) / 1000,'LineWidth',2);
axis ij

idx = find( Time > 0& Time < 80000 );
f = focal_length(idx);
df = abs(diff(f));
df=[df;0]/12


a = aperture(idx);
da = abs(diff(a));
da=[da;0]/1.18;

d_x = drone_x(idx);
dx = abs(diff(d_x));
dx = [dx;0];

d_y = drone_y(idx);
dy = abs(diff(d_y));
dy = [dy;0];

d_z = drone_z(idx);
dz = abs(diff(d_z));
dz = [dz;0];

dyp = plot(Time(idx) / 1000,dy(idx),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:10:idx(end),'Color','#006AAF'); % dark blue

hold on
ax = gca;
ax.FontSize = 70; 
%dap= plot(Time(idx) / 1000,da(idx),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:10:idx(end),'Color','#C3AD1E'); % dark yellow

dfp= plot(Time(idx) / 1000,df(idx),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:10:idx(end),'Color','#FF6442'); %dark orange


%lgd = legend([dap,dfp,dyp],{ '|v_A|', '|v_f|','|v_y|'},'Orientation','Horizontal','NumColumns',3);
lgd = legend([dfp,dyp],{ '|v_f|','|v_y|'},'Orientation','Horizontal','NumColumns',3);

lgd.FontSize = 70;

ylabel('Actuators')
xlh = xlabel('T(s)');
xlh.Position(1) = -13; 
xlh.Position(2) = -15; 

xlim([0,90])