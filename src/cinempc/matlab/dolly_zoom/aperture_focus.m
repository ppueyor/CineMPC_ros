
[Time, Sequence, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d, relative_yaw, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_yaw', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});

close all

%idx = find( Time > 65000 & Time < 89000 );

id_seq2 = find( Time > 00000);
idx = find( Time > 0 );
fig = figure('units','normalized','outerposition',[0.25 0 0.75 1]);
left_color = [0 0 0];
right_color = [0 0 0];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);

axes('Fontsize',18)
yyaxis left
ylabel('F(m)')
plot(Time(id_seq2) / 1000, focus_distance(id_seq2)/100,'ks-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:5:idx(end),'Color','#FF6442');
axis([Time(idx(1))/ 1000, Time(idx(end))/ 1000, 0, 70])

ax = gca;
ax.FontSize = 70; 
hold on

xlim([0,90]);


yyaxis right


a = plot(Time(idx) / 1000, aperture(idx),'bo-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:5:idx(end),'Color','#006AAF');
hold on
ylabel('A(f\_stop)')
axis([Time(idx(1))/ 1000, Time(idx(end))/ 1000, 0, 23])


ax = gca;
ax.FontSize = 70; 

lg = legend('F','A', 'd','Orientation','Horizontal')
lg.FontSize = 80;
xlh = xlabel('T(s)');
xlh.Position(1) = -8; 
xlh.Position(2) = 0; 

%plot(Time / 1000, Sequence * 10000,'LineWidth',2);

xlim([0,90]);
