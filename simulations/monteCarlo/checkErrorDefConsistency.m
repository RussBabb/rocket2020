function checkErrorDefConsistency( simpar, x )
%checkErrorDefConsistency checks consistency of the error injection,
%calcuation, and correction functions.
%
% Inputs:
%   simpar = description (units)
%   x = description (units)
%
% Example Usage
% checkErrorDefConsistency( simpar, x )
%

% Author: Randy Christensen
% Date: 25-Jan-2019 12:02:28
% Reference:
% Copyright 2018 Utah State University
fnames = fieldnames(simpar.errorInjection);
delx = zeros(length(fnames),1);
for i=1:length(fnames)
    delx(i) = simpar.errorInjection.(fnames{i});
end
x_errorInject = injectErrors(x,delx)
estimationErrors = calcErrors(x_errorInject, x)
assert(max(estimationErrors - delx) < 1e-5);
x_errorCorrect = correctErrors(x_errorInject, delx)
assert(max(x - x_errorCorrect) < 1e-12);
end
