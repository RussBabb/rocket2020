clearvars
close all
clc
totalTimeId = tic;

addpath('Utilities', 'Rocket Profiles')

%Load resources
% global stdAtm extStdAtmAvg extStdAtmMax extStdAtmMin motorProfile
% load("StandardAtmosphere")
% load("ExtendedAtmosphere")
% load("Ces_M2150")

%% Setup paths and matlab file object for saving data
saveName = ['sim_', datestr(datetime('now'), 'yyyy_mm_dd_HH_MM_ss')];
paramFile = 'config.xlsx';
saveDir = ['.\sims\', saveName, '\'];
mkdir(saveDir)
copyfile(paramFile, [saveDir, saveName, '_config.xlsx']);
filenamepath = [saveDir, saveName, '.mat'];
savefile = matfile(filenamepath);
savefile.savedir = saveDir;
savefile.filename = saveName;

%% Read in the simulation parameters
%Define the simparams
checkMapping = 0;
checkProp = 0;
runSingleSim = 0;
runMonteCarlo = 1;
savefigs = 0;

[simpar, simpar_ref] = createSimParams(paramFile);
simpar.savedir = saveDir;

simpar.general.processAltimeter = 1;
simpar.general.processAirspeed = 0;

N_truthStates = simpar.general.n_truth;
N_navStates = simpar.general.n_nav;

%% Setup Inertia Tensor
simpar.rocket.I_b = [simpar.rocket.I_xx, simpar.rocket.I_xy, simpar.rocket.I_xz;
    simpar.rocket.I_xy, simpar.rocket.I_yy, simpar.rocket.I_yz;
    simpar.rocket.I_xz, simpar.rocket.I_yz, simpar.rocket.I_zz];
simpar_ref.rocket.I_b = simpar.rocket.I_b;

%% Check mapping equations
if checkMapping
    x = [20, 20, 20, 100, 100, 100, 0, 0, 0, 0, 0.00, 0.00, 0.00, 15.0,...
        0.005, 0.005, 0.005, 0.00005, 0.00005, 0.00005, 10, 3, 0.01, 0.01, 0.01]';
    x(7:10) = normalizeQuat([1, 1, 1, 1]');
    checkErrorDefConsistency(simpar, x);
end

%% Check propagation and nonlinear measurement modeling
if checkProp
    simpar_ref.general.processAltimeter = simpar.general.processAltimeter;
    simpar_ref.general.processAirspeed = simpar.general.processAirspeed;
    
    traj_propcheck = runSim(simpar_ref,1,1);
    savefile.traj_propcheck = traj_propcheck;
    h_figs_prop_check = plotNavPropErrors(traj_propcheck);
    if savefigs
        for i = 1:length(h_figs_prop_check)
            figfilename = sprintf('checkProp_%d',i);
            h = h_figs_prop_check(i);
            saveas(h,fullfile(savedir,figfilename),'fig');
            saveas(h,fullfile(savedir,figfilename),'epsc');
            saveas(h,fullfile(savedir,figfilename),'png');
        end
    end
end

%% Run Single Simulation
if runSingleSim
    traj_single_sim = runSim(simpar, true, 1);
    savefile.traj_single_sim = traj_single_sim;
    
    %Create Plots
    hfigs = plotEstimationErrors(traj_single_sim);
    
    %Save Plots
    if savefigs
        disp('Saving plots...')
        for i = 1:length(hfigs)
            figfilename = sprintf('singleSim_%d',i);
            h = hfigs(i);
            saveas(h,fullfile(saveDir,figfilename),'fig');
            saveas(h,fullfile(saveDir,figfilename),'epsc');
            saveas(h,fullfile(saveDir,figfilename),'png');
            saveas(h,fullfile(saveDir,figfilename),'pdf');
        end
        disp('Plots saved.')
    end
end

%% Run Monte Carlo
if runMonteCarlo
    % Preallocated error buffer to enable parallel processing
    nstep = simpar.general.tsim/simpar.general.dt + 1;
    errors = zeros(N_navStates - 1, nstep, simpar.general.N_MonteCarloRuns);
    
    % Run Monte Carlo simulation
    parfor i=1:simpar.general.N_MonteCarloRuns
        traj(i) = runSim(simpar, 0, i);
        errors(:,:,i) = calcEstimationErrorsFromMC( traj(i).navState, ...
            traj(i).truthState );
        fprintf('%d/%d complete\n',i, simpar.general.N_MonteCarloRuns);
    end
    
    % Create Monte Carlo plots
    disp('Monte Carlo simulation complete.  Generating plots...')
    hfigs = plotMonteCarlo(errors, traj(1));
    savefile.errors = errors;
    
    %Save Plots
    if savefigs
        disp('Saving plots...')
        for i = 1:length(hfigs)
            figfilename = sprintf('monteCarlo_%d',i);
            h = hfigs(i);
            saveas(h,fullfile(saveDir,figfilename),'fig');
            saveas(h,fullfile(saveDir,figfilename),'epsc');
            saveas(h,fullfile(saveDir,figfilename),'png');
            saveas(h,fullfile(saveDir,figfilename),'pdf');
        end
        disp('Plots saved.')
    end
end

%% Final stuff
totalTime = toc(totalTimeId)
savefile.totalTime = totalTime;