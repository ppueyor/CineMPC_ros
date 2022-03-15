
[Time, Sequence, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d, relative_yaw, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_yaw', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});

close all
figure('units','normalized','outerposition',[0.25 0 0.75 1]);
idx = find( Time >= 0 );


plot2=plot(Time(idx) / 1000, yaw_gt(idx) + 1.57,'--', 'LineWidth',10, 'Markersize',12, 'MarkerIndices',1:2:idx(end),'Color','#FFC075'); %light orange
hold on
plot4=plot(Time(idx) / 1000, pitch_gt(idx),'--', 'LineWidth',10, 'Markersize',2, 'MarkerIndices',1:2:idx(end),'Color','#75B2FF'); %light blue
plot1=plot(Time(idx) / 1000, yaw_perception(idx),'o-', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:10:idx(end),'Color','#FF6442'); %dark orange
ax = gca;
ax.FontSize = 70; 


%plot2=plot(Time(idx) / 1000, JDoF(idx),'s-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:5:idx(end),'Color','#006AAF'); % dark blue

plot3=plot(Time(idx) / 1000, pitch_perception(idx),'o-', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:10:idx(end),'Color','#006AAF'); %dark blue

hold on


ylim([-1.8 0.5])

lgd = legend([plot1 plot2, plot3, plot4],{'Yaw_{est}', 'Yaw_{gt}', 'Pitch_{est}','Pitch_{gt}'},'Orientation','Vertical');
lgd.FontSize = 70;

ax = gca;
ax.FontSize = 70; 
ax.YColor = 'k'

ylabel('R_p, R_{p.est}(RPY-rad)')
xlh = xlabel('T(s)');
xlh.Position(1) = -8; 
xlh.Position(2) = -22; 

xlim([0,105])