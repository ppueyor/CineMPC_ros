
[Time, Sequence, v_x_kf, v_y_kf, v_z_kf, v_x_gt, v_y_gt, v_z_gt] = csvimport('log_file.csv', 'columns', {'Time', 'Sequence', 'v_x_kf', 'v_y_kf', 'v_z_kf', 'v_x_gt', 'v_y_gt', 'v_z_gt'});

close all
figure('units','normalized','outerposition',[0.25 0 0.75 1]);
idx = find( Time >= 0 );


vx_gt=plot(Time(idx) / 1000 ,v_x_gt(idx),'-.*', 'LineWidth',3, 'Markersize',15, 'MarkerIndices',1:4:idx(end),'Color','#FFC075'); %light orange
hold on
vx_kf=plot(Time(idx) / 1000, v_x_kf(idx),'o-', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:10:idx(end),'Color','#FF6442'); %dark orange
ax = gca;
ax.FontSize = 70; 

vy_gt=plot(Time(idx) / 1000 ,v_y_gt(idx),'-.*', 'LineWidth',3, 'Markersize',15, 'MarkerIndices',1:4:idx(end),'Color','#75B2FF'); %light blue
vy_kf=plot(Time(idx) / 1000, v_y_kf(idx),'o-', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:10:idx(end),'Color','#006AAF'); %dark blue

ylabel('v_{plane}(m/s)')
hold on

vz_gt=plot(Time(idx) / 1000 ,v_z_gt(idx),'-.*', 'LineWidth',3, 'Markersize',15, 'MarkerIndices',1:4:idx(end),'Color','#EADA75'); %light yellow
vz_kf=plot(Time(idx) / 1000, v_z_kf(idx),'o-', 'LineWidth',12, 'Markersize',12, 'MarkerIndices',1:10:idx(end),'Color','#C3AD1E'); %darl yellow

%plot4=plot(Time(idx) / 1000, pitch_gt(idx),'--', 'LineWidth',10, 'Markersize',2, 'MarkerIndices',1:2:idx(end),'Color','#75B2FF'); %light blue


lgd = legend([vx_kf, vy_kf, vz_kf,vx_gt, vy_gt, vz_gt],{'vx_{est}', 'vy_{est}', 'vz_{est}','vx_{gt}', 'vy_{gt}', 'vz_{gt}'},'Orientation','Horizontal','NumColumns',3);
lgd.FontSize = 70;

ax = gca;
ax.FontSize = 70; 
ax.YColor = 'k'

xlh = xlabel('T(s)');
xlh.Position(1) = -8; 
xlh.Position(2) = -22; 

xlim([0,105])