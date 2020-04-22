function h_figs = plotEstimationErrors(traj)
%PLOTESTIMATIONERRORS_GPSINS plots residuals and estimation errors for a
%GPS/INS

% Extract true states, nav states, and covariances
truthState = traj.truthState;
navState = traj.navState;
navCov = traj.navCov;
h_figs = [];

% traj.time_kalman = traj.time_kalman/3600;
% traj.time_nav = traj.time_nav/3600;

%Plot trajectory
h_figs(end+1) = figure;
plot3(truthState(2,:)',...
    truthState(1,:)',...
    -truthState(3,:)');
hold on
plot3(navState(2,:)',...
    navState(1,:)',...
    -navState(3,:)');
xlabel('East (m)')
ylabel('North (m)')
zlabel('Altitude (m)')
legend('Truth Trajectory', 'Navigation Estimate')
title('Trajectory Propagation')
grid on;
axis equal;

%Plot Altimeter residuals
h_figs(end+1) = figure; %#ok<*AGROW>
stairs(traj.time_kalman,traj.navRes_alt); hold on
stairs(traj.time_kalman,...
    3.*sqrt(squeeze(traj.navResCov_alt)),'r--');
stairs(traj.time_kalman,...
    -3.*sqrt(squeeze(traj.navResCov_alt)),'r--');
xlabel('Time (sec)')
ylabel('Altimeter Residuals (m)')
set(gca, 'FontSize', 12)
grid on;

%Plot Airspeed residuals
h_figs(end+1) = figure; %#ok<*AGROW>
stairs(traj.time_kalman,traj.navRes_air); hold on
stairs(traj.time_kalman,...
    3.*sqrt(squeeze(traj.navResCov_air)),'r--');
stairs(traj.time_kalman,...
    -3.*sqrt(squeeze(traj.navResCov_air)),'r--');
xlabel('Time  (sec)')
ylabel('Airspeed Residuals (m/s)')
set(gca, 'FontSize', 12)
grid on;

%Plot mahalanobis distance
% [dof,n] = size(traj.navRes);
% mdist_2 = zeros(n,1);
% P_threshold = 0.95;
% for i=1:n
%     mdist_2(i) = traj.navRes(:,i)'*(traj.navResCov(:,:,i)\traj.navRes(:,i));
% end
% h_figs(end+1) = figure;
% stairs(traj.time_kalman, mdist_2)
% hold all
% stairs(traj.time_kalman, ones(n,1) * chi2inv(P_threshold,dof),'r--')
% legend('d^2_{mahalanobis}',sprintf('P_{threshold} = %g',P_threshold))
% xlabel('Time  (hours)')
% grid on;

%Rocket Position estimates
axis_str = {'X','Y','Z'};
for i=1:3
    h_figs(end+1) = figure;
    stairs(traj.time_nav, [navState(i,:)', truthState(i,:)']); hold on
    stairs(traj.time_nav, navState(i,:)' + 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav, navState(i,:)' - 3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time  (sec)')
    ystring = sprintf('Rocket %s Position Estimate (m)',axis_str{i});
    ylabel(ystring)
    legend('Estimated', 'True', '3-sigma')
    grid on;
end

%Rocket Position errors
axis_str = {'X','Y','Z'};
for i=1:3
    h_figs(end+1) = figure;
    stairs(traj.time_nav, navState(i,:)' - truthState(i,:)'); hold on
    stairs(traj.time_nav, 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav,-3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time  (sec)')
    ystring = sprintf('Rocket %s Position Est Err (m)',axis_str{i});
    ylabel(ystring)
    legend('Error','3-sigma')
    grid on;
end

%Rocket Velocity estimates
axis_str = {'X','Y','Z'};
for i=4:6
    h_figs(end+1) = figure;
    stairs(traj.time_nav,[navState(i,:)',truthState(i,:)']); hold on
    stairs(traj.time_nav,navState(i,:)' + 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav,navState(i,:)' - 3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time  (sec)')
    ystring = sprintf('Rocket %s Velocity Estimate (m/s)',axis_str{i-3});
    ylabel(ystring)
    legend('Estimated','True','3-sigma')
    grid on;
end

%Rocket Velocity errors
axis_str = {'X','Y','Z'};
for i=4:6
    h_figs(end+1) = figure;
    stairs(traj.time_nav,navState(i,:)' - truthState(i,:)'); hold on
    stairs(traj.time_nav, 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav,-3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time  (sec)')
    ystring = sprintf('Rocket %s Velocity Est Err (m/s)',axis_str{i-3});
    ylabel(ystring)
    legend('error','3-sigma')
    grid on;
end

%Rocket Attitude errors
axis_str = {'X','Y','Z'};
[ rotVector, ~] = calcAttitudeError(navState(7:10,:),...
    truthState(7:10,:));
for i=7:9
    h_figs(end+1) = figure;
    stairs(traj.time_nav,rotVector(i-6,:)); hold on
    stairs(traj.time_nav, 3.*sqrt(squeeze(navCov(i,i,:))),'r--');
    stairs(traj.time_nav, -3.*sqrt(squeeze(navCov(i,i,:))),'r--'); hold off
    xlabel('Time  (sec)')
    ystring = sprintf('Rocket %s Attitude Est Err (rad)',axis_str{i-6});
    ylabel(ystring)
    legend('estimated','true','3-sigma')
    grid on;
end

%Accel Bias Estimates
axis_str = {'X_b','Y_b','Z_b'};
for i=11:13
    h_figs(end+1) = figure;
    stairs(traj.time_nav,[navState(i,:)',truthState(i+4,:)']); hold on
    stairs(traj.time_nav,navState(i,:)' + 3.*sqrt(squeeze(navCov(i-1,i-1,:))),'r--');
    stairs(traj.time_nav,navState(i,:)' - 3.*sqrt(squeeze(navCov(i-1,i-1,:))),'r--'); hold off
    xlabel('Time  (sec)')
    ystring = sprintf('%s Accel Bias Estimate (m/s^2)',axis_str{i-10});
    ylabel(ystring)
    legend('estimated','true','3-sigma')
    grid on;
end

%Gyro Bias Estimates
axis_str = {'X_b','Y_b','Z_b'};
for i=14:16
    h_figs(end+1) = figure;
    stairs(traj.time_nav,[navState(i,:)',truthState(i+4,:)']); hold on
    stairs(traj.time_nav,navState(i,:)' + 3.*sqrt(squeeze(navCov(i-1,i-1,:))),'r--');
    stairs(traj.time_nav,navState(i,:)' - 3.*sqrt(squeeze(navCov(i-1,i-1,:))),'r--'); hold off
    xlabel('Time  (sec)')
    ystring = sprintf('%s Gyro Bias Estimate (rad/s)',axis_str{i-13});
    ylabel(ystring)
    legend('estimated','true','3-sigma')
    grid on;
end

%Altimeter Bias Estimate
h_figs(end+1) = figure;
stairs(traj.time_nav,[navState(17,:)',truthState(21,:)']); hold on
stairs(traj.time_nav,navState(17,:)' + 3.*sqrt(squeeze(navCov(16,16,:))),'r--');
stairs(traj.time_nav,navState(17,:)' - 3.*sqrt(squeeze(navCov(16,16,:))),'r--'); hold off
xlabel('Time  (sec)')
ylabel('Altimeter Bias Estimate (m)')
legend('estimated','true','3-sigma')
grid on;

%Airspeed Bias Estimate
h_figs(end+1) = figure;
stairs(traj.time_nav,[navState(18,:)',truthState(22,:)']); hold on
stairs(traj.time_nav,navState(18,:)' + 3.*sqrt(squeeze(navCov(17,17,:))),'r--');
stairs(traj.time_nav,navState(18,:)' - 3.*sqrt(squeeze(navCov(17,17,:))),'r--'); hold off
xlabel('Time  (sec)')
ylabel('Airspeed Bias Estimate (m/s)')
legend('estimated','true','3-sigma')
grid on;

end