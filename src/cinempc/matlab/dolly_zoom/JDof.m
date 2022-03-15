
[Time, Sequence, world_x_gt, world_y_gt, world_z_gt, world_x_perception, world_y_perception, world_z_perception, world_x_kf, world_y_kf, world_z_kf, d_gt, v_x_kf, v_y_kf, v_z_kf, pitch_gt, yaw_gt, pitch_perception, yaw_perception, focal_length, focus_distance, aperture, dn, df, im_u, im_v_up, im_v_center, im_v_down, cost, Jp, Jim, JDoF, Jf, focal_length_d, dn_d, df_d, relative_yaw, relative_yaw_d, d_d, im_u_d, im_v_up_d, im_v_center_d, im_v_down_d ] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'world_x_gt', 'world_y_gt', 'world_z_gt', 'world_x_perception', 'world_y_perception', 'world_z_perception', 'world_x_kf', 'world_y_kf', 'world_z_kf', 'd_gt', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'pitch_gt', 'yaw_gt', 'pitch_perception', 'yaw_perception', 'focal_length', 'focus_distance', 'aperture', 'dn', 'df', 'im_u', 'im_v_up', 'im_v_center', 'im_v_down', 'cost', 'Jp', 'Jim', 'JDoF', 'Jf', 'focal_length_d', 'dn_d', 'df_d', 'relative_yaw', 'relative_yaw_d', 'd_d', 'im_u_d', 'im_v_up_d', 'im_v_center_d', 'im_v_down_d'});
[Time_bad,JDoF_bad, Jf_bad] = csvimport('log_file_baseline.csv', 'columns', {'Time','JDoF','Jf'});

close all
figure('units','normalized','outerposition',[0.25 0 0.75 1]);
idx = find( Time >= 0 );

title('Image position of u');
xlabel('Time(s)')

%plot1= plot(Time(idx) / 1000, JDoF_bad(idx),'*', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:4:idx(end),'Color','#75B2FF'); %light blue

plot1=plot(Time(idx) / 1000, Jf(idx),'o-', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:3:idx(end),'Color','#FF6442'); %dark orange
hold on
ax = gca;
ax.FontSize = 70; 


ylabel('J_{DoF}, J_{f}')
%plot2=plot(Time(idx) / 1000, JDoF(idx),'s-', 'LineWidth',8, 'Markersize',15, 'MarkerIndices',1:5:idx(end),'Color','#006AAF'); % dark blue

plot3=plot(Time(idx) / 1000, JDoF(idx),'o-', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:4:idx(end),'Color','#006AAF'); %dark blue

hold on

yyaxis right


plot2=plot(Time_bad / 1000, Jf_bad,'--', 'LineWidth',10, 'Markersize',12, 'MarkerIndices',1:2:idx(end),'Color','#FFC075'); %light orange

plot4=plot(Time_bad / 1000, JDoF_bad,'--', 'LineWidth',10, 'Markersize',2, 'MarkerIndices',1:2:idx(end),'Color','#75B2FF'); %light blue


lgd = legend([plot1 plot2, plot3, plot4],{'J_f', 'J_{f.base}', 'J_{DoF}','J_{DoF.base}'},'Orientation','Horizontal','NumColumns',2);
lgd.FontSize = 80;

axis([Time(idx(1))/ 1000, Time(idx(end))/ 1000, 0, 110000])
ax = gca;
ax.FontSize = 70; 
ax.YColor = 'k'
ax.SortMethod = 'childorder';

xlim([0 90])

ylabel('J_{DoF.base}, J_{f.base}')
xlh = xlabel('T(s)');
xlh.Position(1) = -8; 
xlh.Position(2) = -22; 