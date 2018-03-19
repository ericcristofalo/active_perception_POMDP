%--------------------------------------------------------------------------
%
% File Name:      simulate.m
% Date Created:   2018/03/14
% Date Modified:  2018/03/16
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Generates value given current state, current history, 
%                 and desired search depth. 
%
% Inputs:         s: 
%
% Outputs:        value: 
%
%--------------------------------------------------------------------------

%% Simulate
function value = simulate(s, d, n_cur, a_prev, o_prev, sys)

   % Declare Global Variables
   global nInd g E C N B Mu Sigma W Q gamma alpha_a alpha_o c k_a k_o

   % End Simulate Function
   if ( d==0 )
      value = 0;
      return;
   end

   % Graph Structure
   VariableNames = {'index','type'};

   % Action Progressive Widening ------------------------------------
   if ( size(C{n_cur},2) <= k_a*N(n_cur)^alpha_a )
      % Select Action (Heuristic: Sample About Previous Action)
      a_new = mvnrnd(a_prev,eye(3)*0.001)';
      % Add Action to Tree
      nInd = nInd+1; % increase total node count
      n_new_a = nInd; % assign new node index
      g = addnode(g,table({n_new_a},{'a'},'VariableNames',VariableNames)); % add new node to tree
      g = addedge(g,n_cur,n_new_a); % add new edge to tree
      % Record/Initialize Action
      C{n_cur} = [C{n_cur},n_new_a]; % child of node index n_cur
      C{n_new_a} = []; % initialize child of node index n_new_a
      E{n_new_a} = a_new; % new action for node index n_new_a
      N(n_new_a) = 1; % initialize num visits for node index n_new_a
   end
   % Select Action From All Children of Current Node
   % Note: action does not affect value at node (only observations will)
   cost = -1E6;
   cost_temp = 0;
   n_temp = 0;
   for a = 1:size(C{n_cur},2)
      a_ind = C{n_cur}(a);
      cost_temp = c*sqrt(log(N(n_cur))/N(a_ind));
      if ( cost_temp>cost )
         cost = cost_temp;
         n_temp = C{n_cur}(a);
      end
   end
   n_cur_a = n_temp; % selected action from Action Progressive Widening
   a_cur = E{n_cur_a};

   % Generate Observation
   s_prime = updateState(s,a_cur,sys);
   o_new = getObservation(s_prime,sys);
   mu_cur = Mu{n_cur};
   sigma_cur = Sigma{n_cur};
   [mu_new,sigma_new] = updateBelief(mu_cur,sigma_cur,a_cur,o_new,sys);
   r = -trace(sigma_new);

   % Observation Progressive Widening -------------------------------
   if ( size(C{n_cur_a},2) <= k_o*N(n_cur_a)^alpha_o )
      % Add Observation to Tree
      nInd = nInd+1; % increase total node count
      n_new_o = nInd; % assign new node index
      g = addnode(g,table({n_new_o},{'o'},'VariableNames',VariableNames)); % add new node to tree
      g = addedge(g,n_cur_a,n_new_o); % add new edge to tree
      % Record/Initialize Observation
      N(n_new_o) = 1;
      C{n_cur_a} = [C{n_cur_a},n_new_o]; % child of node index n_cur_a
      C{n_new_o} = []; % initialize child of node index n_new_o
      E{n_new_o} = o_new; % new action for node index n_new_o
      B{n_new_o} = [];
      W{n_new_o} = [];
      Q{n_new_o} = 0;
      Mu{n_new_o} = mu_new;
      Sigma{n_new_o} = sigma_new;
      % Rollout with Default Policy (Heuristic: same action)
      mu_cur_ = mu_new;
      sigma_cur_ = sigma_new;
      s_prime_ = s_prime;
      r_temp = zeros(d,1);
      for d_roll = 1:d
         s_prime_ = updateState(s_prime_,a_cur,sys);
         o_new_ = getObservation(s_prime_,sys);
         [mu_cur_,sigma_cur_] = updateBelief(mu_cur_,sigma_cur_,a_cur,o_new_,sys);
         r_temp(d_roll) = -trace(sigma_cur_);
      end
      r_out = 0;
      for d_roll = 1:size(r_temp,1) % sum backwards
         r_out = r_out + gamma*r_temp(end+1-d_roll,1);
      end
      r = r + gamma*r_out;
      % new node added, end here
      value  = r;
      return;
   else
      % Sample Observation from Children of n_cur_a
      P = zeros(size(C{n_cur_a},2),1);
      for o = 1:size(C{n_cur_a},2)
         P(o) = N(C{n_cur_a}(o))/N(n_cur_a);
      end
      P = P./sum(P);
      n_new_o = C{n_cur_a}(find(rand<cumsum(P),1,'first'));
      % Append State
      B{n_new_o} = [B{n_new_o},s_prime];
      % Measurement Likelihood
      H_likelihood = zeros(sys.m,1);
      for i = 1:(sys.m/2)
         ind1a = 2*i-1:2*i;
         ind2a = 3*i-2:3*i;
         H_likelihood(ind1a,1) = (1/s_prime(ind2a(3)))*sys.H*s_prime(ind2a,1);
      end
      W{n_new_o} = [W{n_new_o},...
         mvnpdf(o_new,H_likelihood,sys.R)*...
         mvnpdf(s_prime,sys.F*s+sys.B*a_cur,sys.Q)];
      % Normalize Weights
      if (sum(W{n_new_o},2)==0)
         W{n_new_o}(1) = 1;
      end
      % Sample s_prime From Weights
      P = zeros(size(B{n_new_o},2),1);
      for s_ind = 1:size(B{n_new_o},2)
         P(s_ind) = W{n_new_o}(s_ind)/sum(W{n_new_o});
      end
      s_prime = B{n_new_o}(:,find(rand<cumsum(P),1,'first'));
      % Calculate New Reward
      o_new = getObservation(s_prime,sys);
      E{n_new_o} = o_new; % new action for node index n_new_o
      mu_cur = Mu{n_cur};
      sigma_cur = Sigma{n_cur};
      [mu_new,sigma_new] = updateBelief(mu_cur,sigma_cur,a_cur,o_new,sys);
      Mu{n_new_o} = mu_new;
      Sigma{n_new_o} = sigma_new;
      r = -trace(sigma_new);
      % Simulate Reward (Only a Function of new Belief)
      value = simulate(s_prime, d-1, n_new_o, a_prev, o_prev, sys);
      total = r + gamma*value;
      % Update Counts
      N(n_cur) = N(n_cur) + 1;
      N(n_cur_a) = N(n_cur_a) + 1;
      % Update Value
      Q{n_new_o} = Q{n_new_o} + (total-Q{n_new_o})/N(n_cur_a); % new value for node index n_new_o
   end

end

