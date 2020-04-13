function [ ytilde ] = contInertialMeas(...
    accel_bias, gyro_bias, ...
    true_accel, true_gyro, accel_noise, gyro_noise)
%Compute corrupted accelerometer and gyro measurements
% nutilde_b = true_accel + accel_bias + accel_noise;
omegatilde_b = true_gyro + gyro_bias + gyro_noise;
% ytilde = [nutilde_b; omegatilde_b];
ytilde = omegatilde_b;
end