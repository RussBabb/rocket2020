function h_figs = plotNavPropErrors_gpsins(traj)

h_figs = [];

%Plot trajectory
h_figs(end+1) = figure;
plot3(traj.truthState(1,:)',...
    traj.truthState(2,:)',...
    traj.truthState(3,:)');
hold on
plot3(traj.navState(1,:)',...
    traj.navState(2,:)',...
    traj.navState(3,:)');
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend('Truth Trajectory', 'Navigation Estimate')
title('Trajectory Propagation')
grid on;
axis equal;

%Plot satellite position error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(1,:)' - traj.navState(1,:)',...
    traj.truthState(2,:)' - traj.navState(2,:)',...
    traj.truthState(3,:)' - traj.navState(3,:)']);
title('Position Error');
xlabel('Time (s)');
ylabel('Error (m)');
legend('X_I','Y_I','Z_I')
grid on;

%Plot satellite velocity error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(4,:)' - traj.navState(4,:)',...
    traj.truthState(5,:)' - traj.navState(5,:)',...
    traj.truthState(6,:)' - traj.navState(6,:)']);
title('Velocity Error');
xlabel('Time (s)');
ylabel('Error (m/s)');
legend('X_I','Y_I','Z_I')
grid on;

%Plot satellite attitude error
[ rotVector, ~] = calcAttitudeError(...
    traj.truthState(7:10,:), traj.navState(7:10,:));
h_figs(end+1) = figure;
stairs(traj.time_nav, rotVector')
title('Attitude Error');
xlabel('Time (s)');
ylabel('Error (rad)');
legend('X','Y','Z')
grid on;

%Plot beacon position error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(14,:)' - traj.navState(11,:)',...
    traj.truthState(15,:)' - traj.navState(12,:)',...
    traj.truthState(16,:)' - traj.navState(13,:)']);
title('Beacon Position Error');
xlabel('Time (s)');
ylabel('Error (m)');
legend('X_b','Y_b','Z_b')
grid on;

%Plot accel bias error
% h_figs(end+1) = figure;
% stairs(traj.time_nav,...
%     [traj.truthState(11,:)'-traj.navState(11,:)',...
%     traj.truthState(12,:)'-traj.navState(12,:)',...
%     traj.truthState(13,:)'-traj.navState(13,:)']);
% title('Accel Bias Error');
% xlabel('time(s)');
% ylabel('m/s^2');
% legend('X_b','Y_b','Z_b')
% grid on;

%Plot gyro bias error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(17,:)'-traj.navState(14,:)',...
    traj.truthState(18,:)'-traj.navState(15,:)',...
    traj.truthState(19,:)'-traj.navState(16,:)']);
title('Gyro Bias Error');
xlabel('Time (s)');
ylabel('Bias Error (rad/s)');
legend('X_b','Y_b','Z_b')
grid on;

%Plot beacon range bias error
h_figs(end+1) = figure;
stairs(traj.time_nav, traj.truthState(20,:)'-traj.navState(17,:)');
title('Range Bias Error');
xlabel('Time (s)');
ylabel('Bias Error (m)');
grid on;

%Plot beacon range-rate bias error
h_figs(end+1) = figure;
stairs(traj.time_nav, traj.truthState(21,:)'-traj.navState(18,:)');
title('Range-rate Bias Error');
xlabel('Time (s)');
ylabel('Bias Error (m/s)');
grid on;

%Plot gravity bias error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(22,:)'-traj.navState(19,:)',...
    traj.truthState(23,:)'-traj.navState(20,:)',...
    traj.truthState(24,:)'-traj.navState(21,:)']);
title('Gravity Bias Error');
xlabel('Time (s)');
ylabel('Bias Error (m/s^2)');
legend('X_I','Y_I','Z_I')
grid on;

%Plot star camera misalignment error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(25,:)'-traj.navState(22,:)',...
    traj.truthState(26,:)'-traj.navState(23,:)',...
    traj.truthState(27,:)'-traj.navState(24,:)']);
title('Star Camera Misalignment Error');
xlabel('Time (s)');
ylabel('Misalignment Error (rad)');
legend('X_b','Y_b','Z_b')
grid on;

%Plot terrain camera misalignment error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(28,:)'-traj.navState(25,:)',...
    traj.truthState(29,:)'-traj.navState(26,:)',...
    traj.truthState(30,:)'-traj.navState(27,:)']);
title('Terrain Camera Misalignment Error');
xlabel('Time (s)');
ylabel('Misalignment Error (rad)');
legend('X_b','Y_b','Z_b')
grid on;

%Plot feature position errors
[n_truth, ~] = size(traj.truthState);
n_features = (n_truth - 30)/3;
for i=1:n_features
    h_figs(end+1) = figure;
    stairs(traj.time_nav,...
        [traj.truthState(30+i*3-2,:)' - traj.navState(27+i*3-2,:)',...
        traj.truthState(30+i*3-1,:)' - traj.navState(27+i*3-1,:)',...
        traj.truthState(30+i*3,:)' - traj.navState(27+i*3,:)']);
    title(['Feature ', int2str(i), ' Position Error']);
    xlabel('Time (s)');
    ylabel('Error (m)');
    legend('X_I','Y_I','Z_I')
    grid on;
end

%Plot range residuals
h_figs(end+1) = figure; %#ok<*AGROW>
stairs(traj.time_kalman,traj.navRes_range);
hold on
xlabel('Time (s)')
ylabel('Residuals (m)')
title('Range Residuals')
grid on;

%Plot doppler residuals
h_figs(end+1) = figure; %#ok<*AGROW>
stairs(traj.time_kalman,traj.navRes_doppler);
hold on
xlabel('Time (s)')
ylabel('Residuals (m/s)')
title('Range-rate Residuals')
grid on;

%Plot star camera residuals
h_figs(end+1) = figure;
stairs(traj.time_kalman, traj.navRes_b2i(1,:))
hold on
stairs(traj.time_kalman, traj.navRes_b2i(2,:))
stairs(traj.time_kalman, traj.navRes_b2i(3,:))
title('Attitude residuals');
xlabel('Time (s)');
ylabel('Residuals (rad)');
legend('X','Y','Z')
grid on;

%Plot terrain camera (line of sight) residuals
h_figs(end+1) = figure; %#ok<*AGROW>
stairs(traj.time_kalman,traj.navRes_los(1,:)');
hold on
stairs(traj.time_kalman,traj.navRes_los(2,:)');
xlabel('Time (s)')
ylabel('Residuals')
title('Line of Sight Residuals')
legend('X/Z', 'Y/Z')
grid on;

end