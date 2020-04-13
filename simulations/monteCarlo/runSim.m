function [ traj ] = runSim_gpsIns( simpar, verbose,  seed)
rng(seed);
%RUNSIM Runs a single trajectory given the parameters in simparams
tic;
%*********************** Truth Model Calculations ************************
%Read true measurements from aerodynamics simulation
% data = readFlightSim(simpar.general.flightSimFilename);
% simpar.general.dt = mean(diff(data.time));
% simpar.general.tsim = data.time(end);
nstep = simpar.general.tsim/simpar.general.dt + 1;

%Derive the number of steps in the simulation and the time
nstep_aid = ceil(simpar.general.tsim/simpar.general.dt_kalmanUpdate);
% simpar.general.dt = 1/round(1/simpar.general.dt);
t = (0:nstep-1)'*simpar.general.dt;
t_kalman = (0:nstep_aid)'.*simpar.general.dt_kalmanUpdate;
nstep_aid = length(t_kalman);

%Extract the true accelerometer and gyro measurements
% accel_true = [data.acc_x, data.acc_y, data.acc_z]';
% gyro_true = [data.gyro_x, data.gyro_y, data.gyro_z]';

%Generate inputs to truth state dynamics
Q_a = simpar.truth.params.Q_a;
Q_w = simpar.truth.params.Q_w;
% Q_acc_ecrv  = 2*simpar.truth.params.sig_accel_ss^2/simpar.general.tau_accel;
Q_gyro_ecrv = 2*simpar.truth.params.sig_gyro_ss^2/simpar.general.tau_gyro;
Q_range_ecrv = 2*simpar.truth.params.sig_range_ss^2/simpar.general.tau_range;
Q_doppler_ecrv = 2*simpar.truth.params.sig_doppler_ss^2/simpar.general.tau_doppler;
Q_grav_ecrv = 2*simpar.truth.params.sig_grav_ss^2/simpar.general.tau_grav;
Q_sc_ecrv = 2*simpar.truth.params.sig_sc_ss^2/simpar.general.tau_sc;
Q_tc_ecrv = 2*simpar.truth.params.sig_tc_ss^2/simpar.general.tau_tc;

w_a = randn(3,nstep)*sqrt(Q_a/simpar.general.dt);
w_w = randn(3,nstep)*sqrt(Q_w/simpar.general.dt);
% w_acc_ecrv  = randn(1,nstep)*sqrt(Q_acc_ecrv/simpar.general.dt);
w_gyro_ecrv = randn(3,nstep)*sqrt(Q_gyro_ecrv/simpar.general.dt);
w_range_ecrv = randn(1,nstep)*sqrt(Q_range_ecrv/simpar.general.dt);
w_doppler_ecrv = randn(1,nstep)*sqrt(Q_doppler_ecrv/simpar.general.dt);
w_grav_ecrv = randn(3,nstep)*sqrt(Q_grav_ecrv/simpar.general.dt);
w_sc_ecrv = randn(3,nstep)*sqrt(Q_sc_ecrv/simpar.general.dt);
w_tc_ecrv = randn(3,nstep)*sqrt(Q_tc_ecrv/simpar.general.dt);

u_truth = [w_a;
    w_w;
    w_gyro_ecrv;
    w_range_ecrv;
    w_doppler_ecrv;
    w_grav_ecrv;
    w_sc_ecrv;
    w_tc_ecrv];
% u_truth = zeros(14,nstep);

%Calculate the IMU noise strengths
Q_nu        = simpar.truth.params.vrw^2;
Q_omega     = simpar.truth.params.arw^2;

%Generate the IMU noise samples
n_nu        = randn(3,nstep)*sqrt(Q_nu/simpar.general.dt);
n_omega     = randn(3,nstep)*sqrt(Q_omega/simpar.general.dt);

%Generate the measurement noise samples
nu_range = randn(1,nstep_aid)*simpar.truth.params.sig_meas_range;
nu_doppler = randn(1,nstep_aid)*simpar.truth.params.sig_meas_doppler;
nu_sc = [randn(1,nstep_aid)*simpar.truth.params.sig_meas_scx;
    randn(1,nstep_aid)*simpar.truth.params.sig_meas_scy;
    randn(1,nstep_aid)*simpar.truth.params.sig_meas_scz];
nu_tc = [randn(1, nstep_aid)*simpar.truth.params.sig_meas_tcx;
    randn(1, nstep_aid)*simpar.truth.params.sig_meas_tcy];
%*****************End of Truth Model Calculations ************************

%*********************** Misc Calcs **************************************
%Assign frequently-used values
N_truthStates = simpar.general.n_truth;
N_navStates = simpar.general.n_nav;
N_featStates = simpar.general.n_feat;

R_range = simpar.nav.params.sig_meas_range^2;
R_doppler = simpar.nav.params.sig_meas_doppler^2;
R_sc = diag([simpar.nav.params.sig_meas_scx^2,...
    simpar.nav.params.sig_meas_scy^2,...
    simpar.nav.params.sig_meas_scz^2]);
R_tc = diag([simpar.nav.params.sig_meas_tcx^2,...
    simpar.nav.params.sig_meas_tcy^2]);

MU_MOON = simpar.general.MU;
w_s = [0, sqrt(MU_MOON/simpar.init.a_s^3), 0]';
distLimTC = (simpar.init.a_s - simpar.general.R_M)*1.5;
%*********************** End of Misc Calcs *******************************

%************************ Initialization *********************************
%Pre-allocate memory buffers
x_hat_buff      = zeros(N_navStates+N_featStates,nstep);
P_hat_buff      = zeros(N_navStates+N_featStates-1,N_navStates+N_featStates-1,nstep);
res_range       = zeros(1,nstep_aid);
res_doppler     = zeros(1,nstep_aid);
res_b2i         = zeros(3,nstep_aid);
res_los         = zeros(2,nstep_aid);
resCov_range    = zeros(1,nstep_aid);
resCov_doppler  = zeros(1,nstep_aid);
resCov_b2i      = zeros(3,3,nstep_aid);
resCov_los      = zeros(2,2,nstep_aid);
ytilde_buff     = zeros(simpar.general.n_inertialMeas,nstep);
x_buff          = zeros(N_truthStates+N_featStates,nstep);
delx_buff       = zeros(N_navStates+N_featStates-1,nstep);

%Initialize the truth state vector
%The satellite's truth initial position and velocity are set using orbital
%elements. The initial position of the beacon and surface features are set 
%to pre-defined values. The initial sensor biases are set to random values.
[r_sinit, v_sinit] = orbel2rv(simpar.init.a_s, ...
    simpar.init.e_s, ...
    simpar.init.i_s, ...
    simpar.init.W_s, ...
    simpar.init.w_s, ...
    simpar.init.nu_s, ...
    MU_MOON);
x_buff(1:3,1) = r_sinit;
x_buff(4:6,1) = v_sinit;
x_buff(7:10,1) = dcm2q(calc_I2B(r_sinit,v_sinit));
x_buff(11:13,1) = w_s;
x_buff(14:16,1) = [simpar.init.r_bx, simpar.init.r_by, simpar.init.r_bz]';
x_buff(17:19,1) = [simpar.truth.ic.sig_gyrox * randn;
    simpar.truth.ic.sig_gyroy * randn;
    simpar.truth.ic.sig_gyroz * randn];
x_buff(20,1) = simpar.truth.ic.sig_range*randn;
x_buff(21,1) = simpar.truth.ic.sig_doppler*randn;
% x_buff(22:24,1) = [simpar.truth.ic.sig_gravx * randn;
%     simpar.truth.ic.sig_gravy * randn;
%     simpar.truth.ic.sig_gravz * randn];
x_buff(25:27,1) = [simpar.truth.ic.sig_thscx * randn;
    simpar.truth.ic.sig_thscy * randn;
    simpar.truth.ic.sig_thscz * randn];
x_buff(28:30,1) = [simpar.truth.ic.sig_thtcx * randn;
    simpar.truth.ic.sig_thtcy * randn;
    simpar.truth.ic.sig_thtcz * randn];

for f=1:N_featStates/3
    x_buff(N_truthStates+f*3-2:N_truthStates+f*3,1) = [simpar.init.r_fx;
        simpar.init.r_fy;
        simpar.init.r_fz];
end

%Initialize the navigation state vector
%TRICKY; The truth initial variances are used for the position,
%velocity, and attitude states! The bias estimates are initialized to
%zero.
x_hat_buff(1:3,1) = x_buff(1:3,1) ...
    + [simpar.truth.ic.sig_rsx * randn;
    simpar.truth.ic.sig_rsy * randn;
    simpar.truth.ic.sig_rsz * randn];
x_hat_buff(4:6,1) = x_buff(4:6,1) ...
    + [simpar.truth.ic.sig_vsx * randn;
    simpar.truth.ic.sig_vsy * randn;
    simpar.truth.ic.sig_vsz * randn];
delta_q_vec = [0.5*simpar.truth.ic.sig_thx*randn;
    0.5*simpar.truth.ic.sig_thx*randn;
    0.5*simpar.truth.ic.sig_thx*randn];
delta_q = [1; delta_q_vec];
x_hat_buff(7:10,1) = qmult(delta_q, x_buff(7:10,1));
x_hat_buff(7:10,1) = normalizeQuat(x_hat_buff(7:10,1));
x_hat_buff(11:13,1) = x_buff(14:16,1) ...
    + [simpar.truth.ic.sig_rbx * randn;
    simpar.truth.ic.sig_rby * randn;
    simpar.truth.ic.sig_rbz * randn];
% x_hat_buff(22:24,1) = [simpar.truth.ic.sig_gravx * randn;
%     simpar.truth.ic.sig_gravy * randn;
%     simpar.truth.ic.sig_gravz * randn];

for f=1:N_featStates/3
    x_hat_buff(N_navStates+f*3-2:N_navStates+f*3,1) = x_buff(N_truthStates+f*3-2:N_truthStates+f*3,1) ...
        + [simpar.truth.ic.sig_rfx * randn;
        simpar.truth.ic.sig_rfy * randn;
        simpar.truth.ic.sig_rfz * randn];
end

%Inject errors if flag is enabled
if simpar.general.errorPropTestEnable
    fnames = fieldnames(simpar.errorInjection);
    for i=1:length(fnames)
        delx_buff(i,1) = simpar.errorInjection.(fnames{i});
    end
    %x_hat_buff(:,1) = injectErrors(x_hat_buff(:,1), delx_buff(:,1));
    x_hat_buff(:,1) = injectErrors(x_buff(:,1), delx_buff(:,1));
end

%Initialize the navigation state covariance
P_hat_buff(:,:,1) = diag([
    simpar.nav.ic.sig_rsx^2;
    simpar.nav.ic.sig_rsy^2;
    simpar.nav.ic.sig_rsz^2;
    simpar.nav.ic.sig_vsx^2;
    simpar.nav.ic.sig_vsy^2;
    simpar.nav.ic.sig_vsz^2;
    simpar.nav.ic.sig_thx^2;
    simpar.nav.ic.sig_thy^2;
    simpar.nav.ic.sig_thz^2;
    simpar.nav.ic.sig_rbx^2;
    simpar.nav.ic.sig_rby^2;
    simpar.nav.ic.sig_rbz^2;
    simpar.nav.ic.sig_gyrox^2;
    simpar.nav.ic.sig_gyroy^2;
    simpar.nav.ic.sig_gyroz^2;
    simpar.nav.ic.sig_range^2;
    simpar.nav.ic.sig_doppler^2;
    simpar.nav.ic.sig_gravx^2;
    simpar.nav.ic.sig_gravy^2;
    simpar.nav.ic.sig_gravz^2;
    simpar.nav.ic.sig_thscx^2;
    simpar.nav.ic.sig_thscy^2;
    simpar.nav.ic.sig_thscz^2;
    simpar.nav.ic.sig_thtcx^2;
    simpar.nav.ic.sig_thtcy^2;
    simpar.nav.ic.sig_thtcz^2;
    zeros(N_featStates,1);
    %Fill in the rest!
    ]);

for f=1:N_featStates/3
    P_hat_buff(N_navStates+f*3-3,N_navStates+f*3-3) = simpar.nav.ic.sig_rfx^2;
    P_hat_buff(N_navStates+f*3-2,N_navStates+f*3-2) = simpar.nav.ic.sig_rfy^2;
    P_hat_buff(N_navStates+f*3-1,N_navStates+f*3-1) = simpar.nav.ic.sig_rfz^2;
end

%Initialize inertial measurements
ytilde_buff(:,1) = contInertialMeas([0, 0, 0]', ...
    x_buff(17:19,1), ...
    [0, 0, 0]', ...
    w_s,...
    n_nu(:,1), ...
    n_omega(:,1));

%Initialize the measurement counter
k = 1;

% Other intialization
%************************ End of Initialization **************************

for i=2:nstep
    %Propagate the truth states
    [x_buff(:,i), dx_true] = rk4('truthState_de',...
        x_buff(:,i-1),...
        t(i),...
        0,...
        u_truth(:,i),...
        simpar);
    %Create sensor data at tk
    ytilde_buff(:,i) = contInertialMeas([0, 0, 0]', ...
        x_buff(17:19,i), ...
        [0, 0, 0]', ...
        w_s,...
        n_nu(:,i), ...
        n_omega(:,i));
    
    %Propagate the state estimate and covariance from tk-1 to tk using 
    %sensor data from tk-1
    [x_hat_buff(:,i), dx_nav] = rk4('navState_de',...
        x_hat_buff(:,i-1),...
        t(i),...
        ytilde_buff(:,i),...
        0,...
        simpar);
    P_hat_buff(:,:,i) = rk4('navCov_de',...
        P_hat_buff(:,:,i-1),...
        t(i),...
        ytilde_buff(:,i),...
        x_hat_buff(:,i-1),...
        simpar);
%     phi = calc_PHI(x_hat_buff(:,i-1), ytilde_buff(:,i), simpar);
%     B = calc_Bhat(x_hat_buff(:,i-1));
%     Q = calc_Shat_w(simpar);
%     P_hat_buff(:,:,i) = phi*P_hat_buff(:,:,i-1)*phi' + B*Q*B'*simpar.general.dt;
    %Calculate the difference in gravitational acceleration between truth
    %and nav
    x_buff(22:24,i) = dx_nav(4:6,1) - dx_true(4:6,1);
    
    %Propagate the error state from tk-1 to tk if testing flag is enabled
    if simpar.general.errorPropTestEnable
        delx_buff(:,i) = rk4('errorState_de',...
            delx_buff(:,i-1),...
            t(i),...
            ytilde_buff(:,i),...
            x_hat_buff(:,i-1),...
            simpar);
    end
    
    %If a measurement is available, process it
    if abs(t(i)-t_kalman(k+1)) < simpar.general.dt*0.01
        if simpar.general.injectErrorEnable
            checkErrorDefConsistency( simpar, x_buff(:,i) )
        end
        if simpar.general.errorPropTestEnable
            checkErrorPropagation(x_buff(:,i), x_hat_buff(:,i),...
                delx_buff(:,i));
        end
        k = k + 1;
        
        T_LF2I = calc_I2LCF(t(i), simpar)';
        
        rho = x_buff(1:3,i) - T_LF2I*x_buff(14:16,i);
        ele = pi/2 - acos(dot(rho, T_LF2I*x_buff(14:16,i))/(norm(rho)*norm(x_buff(14:16,i))));
        if ele > deg2rad(0) %Check to make sure that satellite is in sight of beacon
            %Process the doppler measurement
            [z_tilde_doppler, zhat_tilde_doppler, H] = calcMeasurement(x_buff(:,i), x_hat_buff(:,i), nu_doppler(k), t(i), 'doppler', simpar);
            res_doppler(k) = z_tilde_doppler - zhat_tilde_doppler;
             
            [x_hat_buff(:,i), P_hat_buff(:,:,i), resCov_doppler(k)] = kalmanUpdate(x_hat_buff(:,i), P_hat_buff(:,:,i), H, R_doppler, res_doppler(k), simpar.general.processDopplerEnable);
            
            %Process the range measurement
            [z_tilde_range, zhat_tilde_range, H] = calcMeasurement(x_buff(:,i), x_hat_buff(:,i), nu_range(k), t(i), 'range', simpar);
            res_range(k) = z_tilde_range - zhat_tilde_range;
            
            [x_hat_buff(:,i), P_hat_buff(:,:,i), resCov_range(k)] = kalmanUpdate(x_hat_buff(:,i), P_hat_buff(:,:,i), H, R_range, res_range(k), simpar.general.processRangeEnable);
            
            if simpar.general.measPertCheckEnable
                reslin_range = H*delx;
                disp(res_range(k) - reslin_range)
                
                reslin_doppler = H*delx;
                disp(res_doppler(k) - reslin_doppler)
            end
        else
            res_range(k) = 0;
            res_doppler(k) = 0;
        end
        
        %Process the star camera orientation measurement
        [z_tilde_b2i, zhat_tilde_b2i, H] = calcMeasurement(x_buff(:,i), x_hat_buff(:,i), nu_sc(:,k), t(i), 'sc', simpar);
        dq = qmult(z_tilde_b2i, qConjugate(zhat_tilde_b2i));
        res_b2i(1:3,k) = 2*dq(2:4);
        
        [x_hat_buff(:,i), P_hat_buff(:,:,i), resCov_b2i(:,:,k)] = kalmanUpdate(x_hat_buff(:,i), P_hat_buff(:,:,i), H, R_sc, res_b2i(:,k), simpar.general.processSCEnable);
        
        if simpar.general.measPertCheckEnable
            reslin_b2i = H*delx;
            disp(res_b2i(:,k) - reslin_b2i)
        end
        
        l = T_LF2I*x_buff(31:33,i) - x_buff(1:3,i);
        alpha = acos(dot(l, -x_buff(1:3,i))/(norm(l)*norm(x_buff(1:3,i))));
        if alpha < deg2rad(30) && norm(l) < distLimTC %Check to make sure that feature is in camera view
            %Create feature along boresight of camera
            if 0
                T_i2b = q2dcm(x_buff(7:10,i))';
                T_b2c = eye(3);
                T_i2c = T_b2c * T_i2b;
                u_zc_i = T_i2c(3,:)';
                r_f_i = x_buff(1:3,i) + 100*u_zc_i;
                x_buff(31:33,i) = T_LF2I' * r_f_i;
                x_hat_buff(28:30,i) = x_buff(31:33,i);
            end
            %Inject errors
            if simpar.general.measPertCheckEnable
                fnames = fieldnames(simpar.errorInjection);
                delx = zeros(length(fnames), 1);
                for j=1:length(fnames)
                    delx(j,1) = simpar.errorInjection.(fnames{j});
                end
                x_hat_buff(:,i) = injectErrors(x_buff(:,i), delx);
            end

            
            %Process the terrain camera line of sight measurements
            [z_tilde_los, zhat_tilde_los, H] = calcMeasurement(x_buff(:,i), x_hat_buff(:,i), nu_tc(:,k), t(i), 'tc', simpar);
            res_los(1:2,k) = z_tilde_los - zhat_tilde_los;
            
            [x_hat_buff(:,i), P_hat_buff(:,:,i), resCov_los(:,:,k)] = kalmanUpdate(x_hat_buff(:,i), P_hat_buff(:,:,i), H, R_tc, res_los(:,k), simpar.general.processTCEnable);
            
            %Test residuals
            if simpar.general.measPertCheckEnable
                delz_nl = res_los(1:2,k);
                delz_l = H*delx;
                measLinTable.delz_nl = delz_nl;
                measLinTable.delz_l = delz_l;
                measLinTable.difference = delz_nl - delz_l;
                disp(struct2table(measLinTable));
            end
            %Remove errors
            if simpar.general.measPertCheckEnable
                x_hat_buff(:,i) = correctErrors(x_hat_buff(:,i), delx);
            end
        else
            res_los(1:2,k) = zeros(2,1);
        end
        
%         H_p = 
%         resCov(:,:,k) = 
%         K_p = 
%         if ~simpar.general.processPositionEnable
%             K_p = K_p.*0;
%         end
%         delx = 
%         P_hat_buff(:,:,i) = 
%         x_hat_buff(:,i) = 
    end
    
    if verbose && mod(i,100) == 0
        fprintf('%0.1f%% complete\n',100 * t(i)/t(end));
    end
end

if verbose
    fprintf('%0.1f%% complete\n',100 * t(i)/t(end));
end

T_execution = toc;
traj = struct('navState',x_hat_buff,...
    'navCov',P_hat_buff,...
    'navRes_range',res_range,...
    'navRes_doppler',res_doppler,...
    'navRes_b2i', res_b2i,...
    'navRes_los',res_los,...
    'navResCov_range',resCov_range,...
    'navResCov_doppler',resCov_doppler,...
    'navResCov_b2i',resCov_b2i,...
    'navResCov_los',resCov_los,...
    'time_nav',t,...
    'time_kalman',t_kalman,...
    'executionTime',T_execution,...
    'gyro',ytilde_buff(1:3,:),...
    'truthState',x_buff);
end