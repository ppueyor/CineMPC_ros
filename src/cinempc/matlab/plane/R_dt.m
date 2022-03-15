
[Time, Sequence, Sim_time, mpc_dt, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d,relative_roll, relative_pitch, relative_yaw, relative_roll_d, relative_pitch_d, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'Sim_time', 'mpc_dt', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_roll', 'relative_pitch', 'relative_yaw', 'relative_roll_d', 'relative_pitch_d', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});
close all
figure('units','normalized','outerposition',[0.25 0 0.75 1]);
idgirl = find( Time >= 0 );
%plot(960-u_boy(idx),Time(idx) / 1000,'LineWidth',2);
axis ij
ylabel('Pixels');
title('Image position of u');
xlabel('Time(s)')

r_d=plot(Time(idgirl) / 1000 ,relative_roll_d(idgirl),'*--', 'LineWidth',3, 'Markersize',15, 'MarkerIndices',1:4:idgirl(end),'Color','#75B2FF'); %light blue

hold on
ax = gca;
ax.FontSize = 70; 
r = plot(Time(idgirl) / 1000,relative_roll(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:10:idgirl(end),'Color','#006AAF'); % dark blue

p_d=plot(Time(idgirl) / 1000, relative_pitch_d(idgirl),'*--', 'LineWidth',3, 'Markersize',15, 'MarkerIndices',1:4:idgirl(end),'Color','#EADA75'); %light yellow
y_d=plot(Time(idgirl) / 1000, relative_yaw_d(idgirl),'*--', 'LineWidth',3, 'Markersize',15, 'MarkerIndices',1:4:idgirl(end),'Color','#FFC075'); %light orange

y =plot(Time(idgirl) / 1000, relative_yaw(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:10:idgirl(end),'Color','#FF6442'); %dark orange
p= plot(Time(idgirl) / 1000, relative_pitch(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:10:idgirl(end),'Color','#C3AD1E');%dark yellow

lgd = legend([ r, p,y,r_d,p_d,y_d],{ 'Roll','Pitch','Yaw', 'Roll^*','Pitch^*', 'Yaw^*'},'Orientation','Horizontal','NumColumns',3);
lgd.FontSize = 70;

ylabel('R_{dt}, R_{dt}^*(RPY-rad)')
xlh = xlabel('T(s)');
xlh.Position(1) = -8; 
xlh.Position(2) = -2; 

xlim([0,105])