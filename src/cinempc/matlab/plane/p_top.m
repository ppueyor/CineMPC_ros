
[Time, Sequence, Sim_time, mpc_dt, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d,relative_roll, relative_pitch, relative_yaw, relative_roll_d, relative_pitch_d, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'Sim_time', 'mpc_dt', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_roll', 'relative_pitch', 'relative_yaw', 'relative_roll_d', 'relative_pitch_d', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});
close all
figure('units','normalized','outerposition',[0.25 0 0.75 1]);
idgirl = find( Time >= 0 );
%plot(960-u_boy(idx),Time(idx) / 1000,'LineWidth',2);
axis ij
ylabel('Pixels');
title('Image position of u');
xlabel('Time(s)')

x_gt=plot(Time(idgirl) / 1000 ,world_x_gt(idgirl),'-.*', 'LineWidth',3, 'Markersize',15, 'MarkerIndices',1:4:idgirl(end),'Color','#75B2FF'); %light blue

hold on
ax = gca;
ax.FontSize = 70; 
x_kf = plot(Time(idgirl) / 1000,world_x_kf(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:10:idgirl(end),'Color','#006AAF'); % light blue
y_gt=plot(Time(idgirl) / 1000 ,world_y_gt(idgirl),'-.*', 'LineWidth',3, 'Markersize',15, 'MarkerIndices',1:4:idgirl(end),'Color','#EADA75'); %light yellow
y_kf= plot(Time(idgirl) / 1000,world_y_kf(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:10:idgirl(end),'Color','#C3AD1E'); % dark yellow

z_gt=plot(Time(idgirl) / 1000 ,world_z_gt(idgirl),'-.*', 'LineWidth',3, 'Markersize',15, 'MarkerIndices',1:4:idgirl(end),'Color','#FFC075'); %light orange
z_kf= plot(Time(idgirl) / 1000,world_z_kf(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:10:idgirl(end),'Color','#FF6442'); %dark orange


lgd = legend([x_kf,y_kf,z_kf, x_gt,y_gt,z_gt],{ 'x_{est}', 'y_{est}','z_{est}', 'x_{gt}', 'y_{gt}', 'z_{gt}'},'Orientation','Horizontal','NumColumns',3);



lgd.FontSize = 70;

ylabel('p_{p\_top}(m)')
xlh = xlabel('T(s)');
xlh.Position(1) = -10; 
xlh.Position(2) = -700; 

xlim([0,105])