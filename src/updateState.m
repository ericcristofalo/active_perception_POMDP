%--------------------------------------------------------------------------
%
% File Name:      updateState.m
% Date Created:   2018/03/01
% Date Modified:  2018/03/16
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Simulates one step forward for a linear Gaussian system
%                 with a given initial state and desired action
%
%--------------------------------------------------------------------------

%% Update the State with 1-step Simulation
function s_prime = updateState(s,a,sys)
   v = mvnrnd(zeros(sys.n,1),sys.Q)';
   s_prime = sys.F*s + sys.B*a + v;
end