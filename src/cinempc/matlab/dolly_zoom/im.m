
[Time, Sequence, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d, relative_yaw, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_yaw', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});
close all
figure('units','normalized','outerposition',[0.25 0 0.75 1]);;
idgirl = find( Time >= 0 & Time <= 90000 );
%plot(960-u_boy(idx),Time(idx) / 1000,'LineWidth',2);
axis ij
ylabel('Pixels');
title('Image position of u');
xlabel('Time(s)')

plot1 = plot(Time(idgirl) / 1000 ,im_u_d(idgirl),'*', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:4:idx(end),'Color','#75B2FF'); %light blue
ylim([0 960]);
hold on
ax = gca;
ax.FontSize = 70; 
plot2 = plot(Time(idgirl) / 1000,im_u(idgirl),'o-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:5:idx(end),'Color','#006AAF'); % dark blue

plot3= plot(Time(idgirl) / 1000, im_v_up_d(idgirl),'*', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:4:idx(end),'Color','#FFC075'); %light orange
 plot(Time(idgirl) / 1000, im_v_center_d(idgirl),'*', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:4:idx(end),'Color','#FFC075'); 

plot4 =plot(Time(idgirl) / 1000, im_v_up(idgirl),'bo-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:5:idx(end),'Color','#FF6442');%dark orange
plot(Time(idgirl) / 1000, im_v_center(idgirl),'bo-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:5:idx(end),'Color','#FF6442');
xlim([0 60]);
lgd = legend([plot1 plot2, plot3, plot4],{'im_{t.x}^*', 'im_{t.x}', 'im_{t.y}^*', 'im_{t.y}'},'Orientation','Horizontal','NumColumns',2);
lgd.FontSize = 70;

ylabel('im(px)')
xlh = xlabel('T(s)');
xlh.Position(1) = -8; 
xlh.Position(2) = -2; 



ylim([0,820]);

xlim([0,90]);