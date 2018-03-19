%--------------------------------------------------------------------------
%
% File Name:      getObservation.m
% Date Created:   2018/03/01
% Date Modified:  2018/03/18
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Generates an observation according to the nonlinear 
%                 Gaussian observation model encoded in the structure, sys. 
%
%--------------------------------------------------------------------------

%% Get Observation
function o = getObservation(s,sys)
   w = mvnrnd(zeros(sys.m,1),sys.R)';
   n = sys.m/2;
   o = zeros(sys.m,1);
   for i = 1:n
      ind1a = 2*i-1:2*i;
      ind2a = 3*i-2:3*i;
      z = s(ind2a(3),1);
      o(ind1a,1) = (1/z)*sys.H*s(ind2a,1) + w(ind1a,1);
   end
end