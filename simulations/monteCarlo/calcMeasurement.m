function [z_tilde, zhat_tilde, H] = calcMeasurement(x, xhat, nu, t, type, simpar)
%calcMeasurement Predicts a measurement using the specified measurement
%model

% Unpack states
r_f = x(1:3);
v_b = x(4:6);
q_b2f = x(7:10);
w_b = x(11:13);
m = x(14);
b_accel = x(15:17);
b_gyro = x(18:20);
b_alt = x(21);
b_air = x(22);
v_wind_f = x(23:25);

rhat_f = xhat(1:3);
vhat_b = xhat(4:6);
qhat_b2f = xhat(7:10);
bhat_accel = xhat(11:13);
bhat_gyro = xhat(14:16);
bhat_alt = xhat(17);
bhat_air = xhat(18);

switch type
    case 'altimeter'
        z_tilde = -r_f(3) + b_alt + nu;
        zhat_tilde = -rhat_f(3) + bhat_alt;
        
        H = zeros(1, length(xhat) - 1);
        H(3) = -1;
        
    case 'airspeed'
        z_tilde = norm(v_b) + b_air + nu;
        zhat_tilde = norm(vhat_b) + bhat_air;
        
        H = zeros(1, length(xhat) - 1);
        H(4:6) = vhat_b'/norm(vhat_b);
        
    otherwise
        error('Invalid measurement type')
end
        
        
end

