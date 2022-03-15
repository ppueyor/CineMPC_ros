
[Time, Sequence, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d, relative_yaw, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_yaw', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});

close all
figure('units','normalized','outerposition',[0.25 0 0.75 1]);
idx = find( Time >= 0 );
id_seq2 = find( Time > 26500 );

d_person = dn_d -0.5;
d_cactus_far = d_person +46.4; 
d_cactus_close = d_person - 22.3; 

axis ij
title('Image position of u');
xlabel('Time(s)')

plot1= plot(Time(idx) / 1000, dn_d(idx),'*', 'LineWidth',12, 'Markersize',11, 'MarkerIndices',1:4:idx(end),'Color','#75B2FF'); %light blue

hold on
ax = gca;
ax.FontSize = 70; 
plot2=plot(Time(idx) / 1000, dn(idx),'s-', 'LineWidth',8, 'Markersize',12, 'MarkerIndices',1:7:idx(end),'Color','#006AAF'); % dark blue

plot5=plot(Time(id_seq2) / 1000, df_d(id_seq2),'*', 'LineWidth',12, 'Markersize',11, 'MarkerIndices',1:4:idx(end),'Color','#FFC075'); %light orange
plot3=plot(Time(id_seq2) / 1000, df(id_seq2),'s-', 'LineWidth',8, 'Markersize',12, 'MarkerIndices',1:7:idx(end),'Color','#FF6442'); %dark orange

plot4=plot(Time(idx) / 1000, d_cactus_far(idx),'bo-', 'LineWidth',8, 'Markersize',12, 'MarkerIndices',1:7:idx(end),'Color','#306D00'); %dark green
plot(Time(idx) / 1000, d_cactus_close(idx),'bo-', 'LineWidth',8, 'Markersize',12, 'MarkerIndices',1:7:idx(end),'Color','#306D00'); %dark green

plot([27 27],[-50, 100],'--k','LineWidth',8)

lgd = legend([plot1, plot2, plot4, plot5, plot3],{'D_n^*', 'D_n','d_{cac}','D_f^*', 'D_f'},'Orientation','Horizontal','NumColumns',3);
lgd.FontSize = 70;


ylabel('DoF(m)')
xlh = xlabel('T(s)');
xlh.Position(1) = -8; 
xlh.Position(2) = -22; 

xlim([0,90]);