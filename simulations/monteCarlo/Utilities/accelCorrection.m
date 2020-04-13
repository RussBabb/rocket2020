function [A_x,A_y,A_z] = accelCorrection(A_xIMU,A_yIMU,A_zIMU,p,q,r)
%ACCELCORRECTION Corrects measured acceleration values for IMU offset from
%the rocket's center of gravity
%   Detailed explanation goes here
global g_0 x_IMU y_IMU z_IMU

A_x = A_xIMU + ((r^2 + q^2)*x_IMU - (p*q - r)*y_IMU - (r*p + q)*z_IMU)/g_0;
A_y = A_yIMU + (-(p*q + r)*x_IMU + (p^2 + r^2)*y_IMU - (r*q - p)*z_IMU)/g_0;
A_z = A_zIMU + (-(r*p - q)*x_IMU - (r*q + p)*y_IMU + (q^2 + p^2)*z_IMU)/g_0;
end

