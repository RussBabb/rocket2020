function h_figs = plotNavPropErrors_gpsins(traj)

truthState = traj.truthState;
navState = traj.navState;
navCov = traj.navCov;
time = traj.time_nav;
h_figs = [];

%Plot trajectory
h_figs(end+1) = figure;
plot3(truthState(2,:)',...
    truthState(1,:)',...
    -truthState(3,:)');
hold on
plot3(traj.navState(2,:)',...
    traj.navState(1,:)',...
    -traj.navState(3,:)');
xlabel('East (m)')
ylabel('North (m)')
zlabel('Altitude (m)')
legend('Truth Trajectory', 'Navigation Estimate')
title('Trajectory Propagation')
grid on;
axis equal;

%Plot position
h_figs(end+1) = figure;
plot(time, [truthState(1,:)',...
    truthState(2,:)',...
    -truthState(3,:)']);
xlabel('Time (sec)')
ylabel('Position (m/s)')
legend('North', 'East', 'Altitude')
title('Rocket Position')
grid on;

%Plot velocity
h_figs(end+1) = figure;
plot(time, [truthState(4,:)',...
    truthState(5,:)',...
    truthState(6,:)']);
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
legend('X_b', 'Y_b', 'Z_b')
title('Rocket Velocity')
grid on;

%Plot attitude
[m, n] = size(truthState);
attitude = zeros(3,n);
for i=1:n
    attitude(:,i) = dcm2xyz(q2dcm(truthState(7:10,i)));
end
h_figs(end+1) = figure;
plot(time, [attitude(1,:)',...
    attitude(2,:)',...
    attitude(3,:)']);
xlabel('Time (sec)')
ylabel('Angle (rad)')
legend('Bank (\phi)', 'Elevation (\theta)', 'Heading (\psi)')
title('Rocket Attitude')
grid on;

%Plot attitude rates
h_figs(end+1) = figure;
plot(time, [truthState(11,:)',...
    truthState(12,:)',...
    truthState(13,:)']);
xlabel('Time (sec)')
ylabel('Rate (rad/s)')
legend('Roll rate (p)', 'Pitch rate (q)', 'Yaw rate (r)')
title('Rocket Attitude Rates')
grid on;

%Plot mass
h_figs(end+1) = figure;
plot(time, truthState(14,:)');
xlabel('Time (sec)')
ylabel('Mass (kg)')
title('Rocket Mass')
grid on;

%Plot position error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(1,:)' - traj.navState(1,:)',...
    traj.truthState(2,:)' - traj.navState(2,:)',...
    traj.truthState(3,:)' - traj.navState(3,:)']);
title('Position Error');
xlabel('Time (s)');
ylabel('Error (m)');
legend('X_f','Y_f','Z_f')
grid on;

%Plot velocity error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [truthState(4,:)' - navState(4,:)',...
    truthState(5,:)' - navState(5,:)',...
    truthState(6,:)' - navState(6,:)']);
title('Velocity Error');
xlabel('Time (s)');
ylabel('Error (m/s)');
legend('X_b','Y_b','Z_b')
grid on;

%Plot attitude error
[ rotVector, ~] = calcAttitudeError(...
    truthState(7:10,:), navState(7:10,:));
h_figs(end+1) = figure;
stairs(traj.time_nav, rotVector')
title('Attitude Error');
xlabel('Time (s)');
ylabel('Error (rad)');
legend('X','Y','Z')
grid on;

%Plot accel bias error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(15,:)'-traj.navState(11,:)',...
    traj.truthState(16,:)'-traj.navState(12,:)',...
    traj.truthState(17,:)'-traj.navState(13,:)']);
title('Accel Bias Error');
xlabel('time(s)');
ylabel('m/s^2');
legend('X_b','Y_b','Z_b')
grid on;

%Plot gyro bias error
h_figs(end+1) = figure;
stairs(traj.time_nav,...
    [traj.truthState(18,:)'-traj.navState(14,:)',...
    traj.truthState(19,:)'-traj.navState(15,:)',...
    traj.truthState(20,:)'-traj.navState(16,:)']);
title('Gyro Bias Error');
xlabel('Time (s)');
ylabel('Bias Error (rad/s)');
legend('X_b','Y_b','Z_b')
grid on;

%Plot altimeter bias error
h_figs(end+1) = figure;
stairs(traj.time_nav, traj.truthState(21,:)'-traj.navState(17,:)');
title('Altimeter Bias Error');
xlabel('Time (s)');
ylabel('Bias Error (m)');
grid on;

%Plot airspeed bias error
h_figs(end+1) = figure;
stairs(traj.time_nav, traj.truthState(22,:)'-traj.navState(18,:)');
title('Airspeed Bias Error');
xlabel('Time (s)');
ylabel('Bias Error (m/s)');
grid on;

%Plot altimeter residuals
h_figs(end+1) = figure; %#ok<*AGROW>
stairs(traj.time_kalman,traj.navRes_alt);
hold on
xlabel('Time (s)')
ylabel('Residuals (m)')
title('Altimeter Residuals')
grid on;

%Plot airspeed residuals
h_figs(end+1) = figure; %#ok<*AGROW>
stairs(traj.time_kalman,traj.navRes_air);
hold on
xlabel('Time (s)')
ylabel('Residuals (m/s)')
title('Airspeed Residuals')
grid on;

end