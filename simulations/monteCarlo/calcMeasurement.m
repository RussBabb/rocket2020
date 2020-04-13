function [z_tilde, zhat_tilde, H] = calcMeasurement(x, xhat, nu, t, type, simpar)
%calcMeasurement Predicts a measurement using the specified measurement
%model

% Unpack states
r_i = x(1:3);
v_i = x(4:6);
q_b2i = x(7:10);
w_b = x(11:13);
r_beacon = x(14:16);
% b_accel = x(11:13);
b_gyro = x(17:19);
b_range = x(20);
b_doppler = x(21);
b_grav = x(22:24);
th_sc = x(25:27);
th_tc = x(28:30);
r_features = x(31:end);

rhat_i = xhat(1:3);
vhat_i = xhat(4:6);
qhat_b2i = xhat(7:10);
rhat_beacon = xhat(11:13);
% b_accel = x(11:13);
bhat_gyro = xhat(14:16);
bhat_range = xhat(17);
bhat_doppler = xhat(18);
bhat_grav = xhat(19:21);
that_sc = xhat(22:24);
that_tc = xhat(25:27);
rhat_features = xhat(28:end);

I3 = eye(3);
WDOT_MOON = simpar.general.WDOT_MOON;
OMEGA = [0, -WDOT_MOON, 0; WDOT_MOON, 0, 0; 0, 0, 0];

switch type
    case 'range'
        T_LF2I = calc_I2LCF(t, simpar)';
        rho = r_i - T_LF2I*r_beacon;
        rhohat = rhat_i - T_LF2I*rhat_beacon;
        
        z_tilde = norm(rho) + b_range + nu;
        zhat_tilde = norm(rhohat) + bhat_range;
        
        H = zeros(1, length(xhat) - 1);
        H(1:3) = rhohat'/norm(rhohat);
        H(10:12) = -rhohat'/norm(rhohat)*T_LF2I;
        H(16) = 1;
        
    case 'doppler'
        T_LF2I = calc_I2LCF(t, simpar)';
        rho = r_i - T_LF2I*r_beacon;
        rhohat = rhat_i - T_LF2I*rhat_beacon;
        
        vnu = v_i - OMEGA*T_LF2I*r_beacon;
        vnuhat = vhat_i - OMEGA*T_LF2I*rhat_beacon;
        
        z_tilde = dot(vnu, rho)/norm(rho) + b_doppler + nu;
        zhat_tilde = dot(vnuhat, rhohat)/norm(rhohat) + bhat_doppler;
        
        H = zeros(1, length(xhat) - 1);
        H(1:3) = vnuhat'/norm(rhohat)*(I3 - rhohat*rhohat'/norm(rhohat)^2);
        H(4:6) = rhohat'/norm(rhohat);
        H(10:12) = vnuhat'/norm(rhohat)*(rhohat*rhohat'/norm(rhohat)^2 - I3)*T_LF2I - rhohat'/norm(rhohat)*OMEGA*T_LF2I;
        H(17) = 1;
        
    case 'tc'
        T_LF2I = calc_I2LCF(t, simpar)';
        
        T_b2tc = q2dcm([1; -0.5*th_tc])';
        T_i2b = q2dcm(q_b2i)';
        l = T_b2tc*T_i2b*(T_LF2I*r_features - r_i);
        
        T_b2tc = q2dcm([1; -0.5*that_tc])';
        T_i2b = q2dcm(qhat_b2i)';
        lhat = T_b2tc*T_i2b*(T_LF2I*rhat_features - rhat_i);
        
        z_tilde = [l(1)/l(3), l(2)/l(3)]' + nu;
        zhat_tilde = [lhat(1)/lhat(3), lhat(2)/lhat(3)]';
        
        H = zeros(2, length(xhat) - 1);
        H_lhat = [1/lhat(3), 0, -lhat(1)/lhat(3)^2; 0, 1/lhat(3), -lhat(2)/lhat(3)^2];
        H(:,1:3) = -H_lhat*T_b2tc*T_i2b;
        H(:,7:9) = -H_lhat*T_b2tc*T_i2b*vx(T_LF2I*rhat_features - rhat_i);
        H(:,24:26) = -H_lhat*T_b2tc*vx(T_i2b*(T_LF2I*rhat_features - rhat_i));
%         H(:,24:26) = -H_lhat*vx(T_b2tc*T_i2b*(T_LF2I*rhat_features - rhat_i));
        H(:,27:29) = H_lhat*T_b2tc*T_i2b*T_LF2I;

    case 'sc'
        z_tilde = qmult([1; -0.5*th_sc], qmult([1; -0.5*nu], q_b2i));
        zhat_tilde = qmult([1; -0.5*that_sc], qhat_b2i);
        
        H = zeros(3, length(xhat) - 1);
        H(:,7:9) = -I3;
        H(:,21:23) = -I3;
    otherwise
        error('Invalid measurement type')
end
        
        
end

