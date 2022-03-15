
[Time, Sequence, Sim_time, mpc_dt, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d,relative_roll, relative_pitch, relative_yaw, relative_roll_d, relative_pitch_d, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'Sim_time', 'mpc_dt', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_roll', 'relative_pitch', 'relative_yaw', 'relative_roll_d', 'relative_pitch_d', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});
close all
figure('units','normalized','outerposition',[0.25 0 0.75 1]);
idgirl = find( Time >= 0 );
%plot(960-u_boy(idx),Time(idx) / 1000,'LineWidth',2);
axis ij
ylabel('Pixels');
title('Image position of u');
xlabel('Time(s)')
diff_x = world_x_kf - world_x_gt;
diff_y= world_y_perception - world_y_kf;
diff_z = world_z_perception - world_z_kf;

x_gt=plot(Time(idgirl) / 1000 ,diff_x(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:4:idgirl(end),'Color','#75B2FF'); %light blue

hold on
ax = gca;
ax.FontSize = 70; 
y_gt = plot(Time(idgirl) / 1000,diff_y(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:4:idgirl(end),'Color','#C3AD1E'); % light blue
z_gt=plot(Time(idgirl) / 1000 ,diff_z(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:4:idgirl(end),'Color','#FF6442'); %light yellow


lgd = legend([x_gt,y_gt,z_gt],{ 'x', 'y', 'z'},'Orientation','Horizontal','NumColumns',3);



lgd.FontSize = 70;

ylabel('p_{p\_top}(m)')
xlh = xlabel('T(s)');
xlh.Position(1) = -10; 
xlh.Position(2) = -700; 

xlim([0,105])
