function xdot = diffeq_truthState( x, t, ~, u, simpar )
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

% Unpack inputs
n_nu = u(1:3);
n_omega = u(4:6);
w_accel = u(7:9);
w_gyro  = u(10:12);
w_alt = u(13);
w_vel = u(14);
w_wind = u(15:17);

% tau_accel = simpar.general.tau_accel;
tau_accel = simpar.general.tau_accel;
tau_gyro = simpar.general.tau_gyro;
tau_alt = simpar.general.tau_alt;
tau_air = simpar.general.tau_air;
tau_wind = simpar.general.tau_wind;

%Extract constants
MU = simpar.constants.MU;
R_E = simpar.constants.R_E;
R_EQ = simpar.constants.R_EQ;
J2 = simpar.constants.J2;
g_0 = simpar.constants.g_0;

%Extract properties
A_ref = simpar.rocket.A_ref;
c_ref = simpar.rocket.c_ref;
x_cp = simpar.rocket.x_cp;
x_cg = simpar.rocket.x_cg_b;
C_L_0 = simpar.rocket.C_L_0;
C_L_alpha = simpar.rocket.C_L_alpha;
C_L_beta = simpar.rocket.C_L_beta;
% C_D = simpar.rocket.C_D;
% C_m = simpar.rocket.C_m;
I_b = simpar.rocket.I_b;
% F_thrust = simpar.rocket.F_thrust;
I_sp = simpar.rocket.I_sp;
% mdot = simpar.rocket.mdot;

%Calculate dependent parameters
h = -r_f(3);
[~, P_atm, rho_atm, c_atm, mu_atm] = getStdAtmProps(h/1000);
R_B2f = q2dcm(q_b2f);

v_air_b = v_b + R_b2f'*v_wind_f; %calculate relative air-speed vector in body-fixed frame
alpha = atan2(v_air_b(3), v_air_b(1));
beta = atan2(v_air_b(2), v_air_b(1));
vmag = norm(v_air_b);
qbar = 0.5*rho_atm*vmag^2;

C_L = C_L_0 + C_L_alpha*alpha;
C_Y = C_L_0 + C_L_beta*beta; %axi-symmetric rocket, side force behaves just like lift
C_D = calcCD(vmag, simpar.rocket, rho_atm, c_atm, mu_atm);
F_thrust = calcThrust(t, simpar.rocket, P_atm);

% Calculate the body to earth fixed rotation matrix and EF gravity vector
R_b2f = q2dcm(q_b2f);
g = [0; 0; calcGrav(h, simpar.init.lat)];

% Precalculate the last portion of the wdot equation
% Iw = [(I_b(2,2) - I_b(3,3))*w_b(2)*w_b(3) + I_b(1,3)*w_b(1)*w_b(2);
%     (I_b(3,3) - I_b(1,1))*w_b(1)*w_b(3) + I_b(1,3)*(w_b(3)^2 - w_b(1)^2);
%     (I_b(1,1) - I_b(2,2))*w_b(1)*w_b(2) + I_b(1,3)*w_b(2)*w_b(3);];

% Calculate body forces
F_b = [F_thrust + qbar*A_ref*(C_L*sin(alpha) + C_Y*sin(beta) - C_D*cos(alpha)*cos(beta); %7.6.23 of Phillips does not include cos(beta) or C_Y*sin(beta)
    qbar*A_ref*(C_D*sin(beta) - C_Y*cos(beta));
    qbar*A_ref*(-C_L*cos(alpha) - C_D*sin(alpha))];

% Calculate body moments
M_b = [0;
    qbar*A_ref*(x_cg - x_cp)/c_ref*(C_L*cos(alpha) + C_D*sin(alpha));
    qbar*A_ref*(x_cg - x_cp)/c_ref*(C_Y*cos(beta) - C_D*sin(beta))];

% Evaluate differential equations
rdot_f = R_b2f*v_b;
vdot_b = F_b/m - cross(w_b, v_b)  + R_b2f'*g + n_nu;
qdot_b2f = qmult(0.5*q_b2f, [0; w_b]); %Consider changing to eq 11.5.11 from Phillips for real-time efficiency
wdot_b = I_b\(M_b + cross(w_b, I_b*w_b)) + n_omega;

mdot = -F_thrust/(g_0*I_sp);

bdot_accel = -b_accel/tau_accel + w_accel;
bdot_gyro = -b_gyro/tau_gyro + w_gyro;
bdot_alt = 0;
bdot_air = 0;

vdot_wind_f = -v_wind_f/tau_wind + w_wind;

% Package states
xdot = [
    rdot_f;
    vdot_b;
    qdot_b2f;
    wdot_b;
    mdot;
    bdot_accel;
    bdot_gyro;
    bdot_alt;
    bdot_air;
    vdot_wind_f;
    ];
end