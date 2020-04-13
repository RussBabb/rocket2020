function xhat_dot = diffeq_navState(xhat,t,ytilde,~,simpar)
% Unpack states
rhat_i = xhat(1:3);
vhat_i = xhat(4:6);
qhat_b2i = xhat(7:10);
b_accel = x(11:13);
bhat_gyro = xhat(14:16);

%Unpack the inputs
wtilde_b = ytilde(1:3);
% Rhat_b2i = q2dcm(q_b2i);

% tau_accel = simpar.general.tau_accel;
tau_gyro = simpar.general.tau_gyro;
tau_range = simpar.general.tau_range;
tau_doppler = simpar.general.tau_doppler;
tau_grav = simpar.general.tau_grav;
tau_sc = simpar.general.tau_sc;
tau_tc = simpar.general.tau_tc;

MU = simpar.general.MU;
J2 = simpar.general.J2;
R_EQ = simpar.general.R_EQ;
T_I2LF = calc_I2LCF(t, simpar);

%Compute xhat_dot
rhatdot_i = vhat_i;
vhatdot_i = calc_g(rhat_i, [0, 0, 1]', R_EQ, J2, MU) - bhat_grav;
% vhatdot_i = T_I2LF'*get_grav_vec(T_I2LF*rhat_i);
qhatdot_b2i = qmult(0.5*qhat_b2i, [0; wtilde_b - bhat_gyro]);
rhatdot_beacon = zeros(3,1);
% bhatdot_accel = 
bhatdot_gyro = -bhat_gyro/tau_gyro;
bhatdot_range = -bhat_range/tau_range;
bhatdot_doppler = -bhat_doppler/tau_doppler;
bhatdot_grav = -bhat_grav/tau_grav;
thatdot_sc = -that_sc/tau_sc;
thatdot_tc = -that_tc/tau_tc;
rhatdot_features = zeros(length(rhat_features),1);

%
xhat_dot = [
    rhatdot_i;
    vhatdot_i;
    qhatdot_b2i;
    rhatdot_beacon;
    bhatdot_gyro;
    bhatdot_range;
    bhatdot_doppler;
    bhatdot_grav;
    thatdot_sc;
    thatdot_tc;
    rhatdot_features
    ];
end
