function xhat_dot = diffeq_navState(xhat,t,ytilde,~,simpar)
% Unpack states
rhat_f = xhat(1:3);
vhat_b = xhat(4:6);
qhat_b2f = xhat(7:10);
bhat_accel = xhat(11:13);
bhat_gyro = xhat(14:16);
bhat_alt = xhat(17);
bhat_air = xhat(18);

%Unpack the inputs
nutilde_b = ytilde(1:3);
omegatilde_b = ytilde(4:6);

tau_accel = simpar.general.tau_accel;
tau_gyro = simpar.general.tau_gyro;
tau_alt = simpar.general.tau_alt;
tau_air = simpar.general.tau_air;

%Calculate dependent parameters
h = -rhat_f(3);

% Calculate the body to earth fixed rotation matrix and EF gravity vector
Rhat_b2f = q2dcm(qhat_b2f);
g = [0; 0; calcGrav(h, simpar.init.lat)];

% Evaluate differential equations
rhatdot_f = Rhat_b2f*vhat_b;
vhatdot_b = nutilde_b - cross(omegatilde_b, vhat_b)  + Rhat_b2f'*g;
qhatdot_b2f = qmult(0.5*qhat_b2f, [0; omegatilde_b]); %Consider changing to eq 11.5.11 from Phillips for real-time efficiency

bhatdot_accel = -bhat_accel/tau_accel;
bhatdot_gyro = -bhat_gyro/tau_gyro;
bhatdot_alt = 0;
bhatdot_air = 0;

% Package states
xhat_dot = [
    rhatdot_f;
    vhatdot_b;
    qhatdot_b2f;
    bhatdot_accel;
    bhatdot_gyro;
    bhatdot_alt;
    bhatdot_air;
    ];
end
