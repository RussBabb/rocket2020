function [ simparams, simparamsref ] = createSimParams( parameterFile)
%Read in general simulation parameters
table = readtable(parameterFile, 'Sheet', 'general');
[num_params,~] = size(table);
for i=1:num_params
    simparams.general.(string(table{i,1})) = table{i,5};
end

%Read in constants
table = readtable(parameterFile, 'Sheet', 'constantParams');
[num_params,~] = size(table);
for i=1:num_params
    simparams.constants.(string(table{i,1})) = table{i,5};
end

%Read in initial conditions
table = readtable(parameterFile, 'Sheet', 'initialConditions');
[num_params,~] = size(table);
for i=1:num_params
    simparams.init.(string(table{i,1})) = table{i,5};
end

%Read in rocket properties
table = readtable(parameterFile, 'Sheet', 'rocketProps');
[num_params,~] = size(table);
for i=1:num_params
    simparams.rocket.(string(table{i,1})) = table{i,5};
end

%Read in truth state parameters
table = readtable(parameterFile, 'Sheet', 'truthStateParams');
[num_params,~] = size(table);
for i=1:num_params
    simparams.truth.params.(string(table{i,1})) = table{i,5};
end

%Read in truth state initial uncertainty
table = readtable(parameterFile, 'Sheet', 'truthStateInitialUncertainty');
[num_params,~] = size(table);
for i=1:num_params
    simparams.truth.ic.(string(table{i,1})) = table{i,5};
end

%Read in navigation state parameters
table = readtable(parameterFile, 'Sheet', 'navStateParams');
[num_params,~] = size(table);
for i=1:num_params
    simparams.nav.params.(string(table{i,1})) = table{i,5};
end

%Read in navigation state initial uncertainty
table = readtable(parameterFile, 'Sheet', 'navStateInitialUncertainty');
[num_params,~] = size(table);
for i=1:num_params
    simparams.nav.ic.(string(table{i,1})) = table{i,5};
end

%Read error injection
table = readtable(parameterFile, 'Sheet', 'errorInjection');
[num_params,~] = size(table);
for i=1:num_params
    simparams.errorInjection.(string(table{i,1})) = table{i,5};
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