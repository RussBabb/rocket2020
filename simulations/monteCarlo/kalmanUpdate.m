function [xnew, Pnew, resCov] = kalmanUpdate(x, P, H, R, res, update)
%kalmanUpdate Uses the Indirect Extended Kalman Filter equations to update
%the state and covariance estimates using a given measurement geometry
%vector and residual. Also returns the residual covariance.
%   x - state vector
%   P - state covariance matrix
%   H - measurement geometry vector
%   res - measurement residual

%Calculate the residual variance
resCov = H*P*H' + R;

%Compute the Kalman gain
K = P*H'/resCov;

if ~update
    K = K.*0;
end

% K(18:20,:) = K(18:20,:).*0;

%Update the state
delx = K*res;
[~, xnew] = correctErrors(x, delx);

%Update the covariance matrix using Joseph formulation
I = eye(length(delx));
Pnew = (I - K*H)*P*(I - K*H)' + K*R*K';

%Symmetrize covariance matrix (just in case)
Pnew = (Pnew+Pnew')/2.0;

%Compute the new residual and residual variance for plotting only
% [m_est, H] = estimate_measurement(type, t, xnew);
% residual = m - m_est;
% resCov = H*Pnew*H' + R;
end

