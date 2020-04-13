function checkErrorPropagation( x, xhat, delx_linear)
%checkErrorPropagation checks the linear propagation of errors
% Inputs:
%   x = truth state
%   xhat = estimated state
%   delx_linear = errors propagated using linear model
%
% Example Usage
% checkErrorPropagation( x, xhat, delx_linear)

% Author: Randy Christensen
% Date: 25-Jan-2019 12:27:36
% Reference: 
% Copyright 2018 Utah State University

delx_nonlinear = calcErrors(xhat,x);
propError = delx_linear - delx_nonlinear;
results = table(delx_linear, delx_nonlinear, propError);
disp(results)

resultsStruct.data = results;
resultsStruct.tableColLabels = {'\delta\hat{x} Linear','\delta\hat{x} Nonlinear','Difference'};
resultsStruct.dataFormat = {'%.6g',2,'%.4g',1};
resultsStruct.dataNanString = '-';
resultsStruct.tableColumnAlignment = 'c';
resultsStruct.tableBorders = 0;
resultsStruct.booktabs = 1;
resultsStruct.tableCaption = 'Results of the error propagation validation process.';
resultsStruct.tableLabel = 'errorVerResults';
resultsStruct.makeCompleteLatexDocument = 0;
latexTable(resultsStruct);
end
