%--------------------------------------------------------------------------
%
% File Name:      updateBelief.m
% Date Created:   2018/03/01
% Date Modified:  2018/03/18
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Updates the belief for a Extended Kalman filter using
%                 linearlization for the nonlinear pinhole camera model
%
%--------------------------------------------------------------------------

%% Update Belief
function [mu,sigma] = updateBelief(mu_prev,sigma_prev,a,o,sys)
   % Update Prior
   mu_ = sys.F*mu_prev + sys.B*a;
   sigma_ = sys.F*sigma_prev*sys.F' + sys.Q;
   % Generate the Linearized Measurement Function
   n = sys.m/2;
   H_tilde = zeros(sys.m,sys.m);
   hCur = zeros(sys.m,1);
   for i = 1:n
      ind1a = 2*i-1:2*i;
      ind2a = 3*i-2:3*i;
      P = mu_(ind2a,1);
      H_tilde(ind1a,ind2a) = sys.H*...
         [1/P(3), 0, -P(1)/P(3)^2;
         0, 1/P(3), -P(2)/P(3)^2;
         0, 0, 0];
      hCur(ind1a,1) = 1/P(3)*sys.H*P;
   end
   % Update Posterior
   S = inv(H_tilde*sigma_*H_tilde'+sys.R);
   mu = mu_ + sigma_*H_tilde'*S*(o-hCur);
   sigma = sigma_ - sigma_*H_tilde'*S*H_tilde*sigma_;
  
end