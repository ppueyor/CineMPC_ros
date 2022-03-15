
[Time, Sequence, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d, relative_yaw, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_yaw', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});

close all

%idx = find( Time > 65000 & Time < 89000 );

idx = find( Time > 0 );
fig = figure('units','normalized','outerposition',[0.25 0 0.75 1]);
left_color = [0 0 0];
right_color = [0 0 0];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);
axes('Fontsize',18)
yyaxis left
plot(Time(idx) / 1000, focal_length_d(idx),'*', 'LineWidth',10, 'Markersize',10, 'MarkerIndices',1:7:idx(end),'Color','#75B2FF');
hold on
plot_f = plot(Time(idx) / 1000, focal_length(idx),'o-', 'LineWidth',12, 'Markersize',10, 'MarkerIndices',1:7:idx(end),'Color','#006AAF');
hold on
ylabel('f(mm)')
axis([Time(idx(1))/ 1000, Time(idx(end))/ 1000, 0, 600])
%hold on

%xlim([29,55]);


yyaxis right
ylabel('d_{dt}(m)')
plot(Time(idx) / 1000, d_gt(idx),'s-', 'LineWidth',12, 'Markersize',10, 'MarkerIndices',1:7:idx(end),'Color','#FF6442');
axis([Time(idx(1))/ 1000, Time(idx(end))/ 1000, 0, 60])


ax = gca;
ax.FontSize = 70; 
lg = legend('f*','f', 'd_{dt}','Orientation','Horizontal')
lg.FontSize = 70;
xlh = xlabel('T(s)');
xlh.Position(1) = -10; 
xlh.Position(2) = 0; 

xlim([0,90]);


%plot(Time / 1000, Sequence * 10000,'LineWidth',2);

