clearvars
close all
clc
totalTimeId = tic;

addpath('Utilities')

%Load resources
global stdAtm extStdAtmAvg extStdAtmMax extStdAtmMin
load("StandardAtmosphere")
load("ExtendedAtmosphere")

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
checkMapping = 1;
checkProp = 0;
runSingleSim = 0;
runMonteCarlo = 0;
savefigs = 0;

[simpar, simpar_ref] = createSimParams(paramFile);
simpar.savedir = saveDir;

simpar.general.processAltimeter = 0;
simpar.general.processAirspeed = 0;

%% Check mapping equations
if checkMapping
    x = [20, 20, 20, 100, 100, 100, 0, 0, 0, 0, 0.00, 0.00, 0.00, 15.0, 0.005, 0.005, 0.005, 0.00005, 0.00005, 0.00005, 10, 3, 0.01, 0.01, 0.01]';
    x(7:10) = normalizeQuat([1, 1, 1, 1]');
    checkErrorDefConsistency(simpar, x);
end

%% Check propagation and nonlinear measurement modeling
if checkProp
    simpar_ref.general.processRangeEnable = 0;
    simpar_ref.general.processDopplerEnable = 0;
    simpar_ref.general.processSCEnable = 0;
    simpar_ref.general.processTCEnable = 0;
    traj_propcheck = runSim_gpsIns(simpar_ref,1,1);
    savefile.traj_propcheck = traj_propcheck;
    h_figs_prop_check = plotNavPropErrors_gpsins(traj_propcheck);
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
    hfigs = plotSingleSim(traj_single_sim);
    
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
    data = readFlightSim(simpar.general.flightSimFilename);
    nstep = simpar.general.tsim/simpar.general.dt + 1;
    errors = zeros(N_navStates + N_featStates - 1, nstep, ...
        simpar.general.N_MonteCarloRuns);
    
    % Run Monte Carlo simulation
    parfor i=1:simpar.general.N_MonteCarloRuns
        traj(i) = runSim_gpsIns(simpar, 0, i);
        errors(:,:,i) = calcEstimationErrorsFromMC( traj(i).navState, ...
            traj(i).truthState );
        fprintf('%d/%d complete\n',i, simpar.general.N_MonteCarloRuns);
    end
    
    % Create Monte Carlo plots
    disp('Monte Carlo simulation complete.  Generating plots...')
    hfigs = plotMonteCarlo_gpsins(errors, traj(1));
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