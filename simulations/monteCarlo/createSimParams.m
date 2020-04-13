function [ simparams, simparamsref ] = createSimParams( parameterFile)
%Read in general simulation parameters
[~,names,raw] = xlsread(parameterFile,'general');
[num_params,~] = size(raw);
for i=2:num_params
    simparams.general.(names{i}) = raw{i,5};
end

%Read in initial conditions
[~,names,raw] = xlsread(parameterFile,'initialConditions');
[num_params,~] = size(raw);
for i=2:num_params
    simparams.init.(names{i}) = raw{i,5};
end

%Read in truth state parameters
[~,names,raw] = xlsread(parameterFile,'truthStateParams');
[num_params,~] = size(raw);
for i=2:num_params
    simparams.truth.params.(names{i}) = raw{i,5};
end

%Read in truth state initial uncertainty
[~,names,raw] = xlsread(parameterFile,'truthStateInitialUncertainty');
[num_params,~] = size(raw);
for i=2:num_params
    simparams.truth.ic.(names{i}) = raw{i,5};
end

%Read in navigation state parameters
[~,names,raw] = xlsread(parameterFile,'navStateParams');
[num_params,~] = size(raw);
for i=2:num_params
    simparams.nav.params.(names{i}) = raw{i,5};
end

%Read in navigation state initial uncertainty
[~,names,raw] = xlsread(parameterFile,'navStateInitialUncertainty');
[num_params,~] = size(raw);
for i=2:num_params
    simparams.nav.ic.(names{i}) = raw{i,5};
end

%Read error injection
[~,names,raw] = xlsread(parameterFile,'errorInjection');
[num_params,~] = size(raw);
for i=2:num_params
    simparams.errorInjection.(names{i}) = raw{i,5};
end

%Create reference trajectory simparams (with all process noise, initial
%conditions, and measurement noise turned off)
simparamsref = simparams;
truthInitialUncertainty = fieldnames(simparamsref.truth.ic);
for i=1:length(truthInitialUncertainty)
    simparamsref.truth.ic.(truthInitialUncertainty{i}) = 0;
end
truthParams = fieldnames(simparamsref.truth.params);
for i=1:length(truthParams)
    simparamsref.truth.params.(truthParams{i}) = 0;
end
end