%--------------------------------------------------------------------------
%
% File Name:      mcts_dpw.m
% Date Created:   2018/03/01
% Date Modified:  2018/03/16
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Generates action given current belief N(mu,sigma) and 
%                 a desired search depth. 
%                 Intended for Monte Carlo Tree Search (MCTS) with 
%                 Double Progressive Widening (DPW) in a belief-MDP. 
%                 I.e., the state is a belief state, which requires 
%                 a generative model for the 
%                    belief update (updateBelief.m) and 
%                    state update (updateState.m). 
%
%                 Based on POMCPOW implementation:
%                 Z. Sunberg and M. Kochenderfer, 
%                 ?POMCPOW: An online algorithm for POMDPs with 
%                 continuous state, action, and observation spaces,? 
%                 arXiv preprint arXiv:1709.06196, 2017.
%
% Inputs:         mu: current mean in R^n
%                 sigma current covariance in R^{nxn}
%                 sys: structure with the system's definition
%                 d: search depth scalar
%
% Outputs:        a: output action in R^m
%
%--------------------------------------------------------------------------

%% Monte Carlo Tree Search with Double Progressive Widening
function a_output = mcts_dpw(mu_in,sigma_in,a_prev,o_prev,sys,d_desired)

   % Declare Global Variables
   global nInd g E C N B Mu Sigma W Q gamma alpha_a alpha_o c k_a k_o

   % Initialize Parameters
   % Graph Structure
   VariableNames = {'index','type'};
   % MCTS
   numSimulations = 100; % desired number of MCTS simulations (could also be for specified computational time)
   gamma = 0.95;  % gamma in (0,1) discout factor
   g = graph;     % tree where indices map to either (actions/observations)
   nInd = 1;      % max number of nodes in tree so far
   n_cur = 1;     % current node index
   g = addnode(g,table({1},{'b'},'VariableNames',VariableNames));
%    A_ind = cell(d_desired,1);	% cell containing action node indices at d^th level of tree: A{d} = [2,5,10,100,...]
%    O_ind = cell(d_desired,1);	% cell containing observation node indices at d^th level of tree: A{d} = [24,28,80,98,...]
   E = {};	% entry (actions or observation) of node: E{i} = action/observation at i^th node
   C = {};	% list of children of node: C{i} = vector of children indices at i^th node
   C{n_cur} = [];
   N = [];  % number of node visits to node: N{i} = number of visits to i^th node
   N(n_cur) = 0;
   B = {};  % list of states associated with a node: B{i} = R^(n x NumOfStates) matrix i^th node's states
   B{n_cur} = [];
   Mu = {};  % mean vector for node: B{i} = i^th node's mean vector
   Mu{n_cur} = mu_in;
   Sigma = {};  % sigma matrix for node: B{i} = i^th node's sigma matrix
   Sigma{n_cur} = sigma_in;
   W = {};	% list of weights associated to states in B: W{i} => i^th node of graph Q
   W{n_cur} = [];
   Q = {};	% list of values: Q{i} => i^th node of graph Q
   Q{n_cur} = -trace(sigma_in);
   % DPW
   alpha_a = 0.5;   % alpha in [0,1]: 
   alpha_o = 0.5;   % alpha in [0,1]: 
   c = 0.5;       % c>0: explore vs. exploit parameter
   k_a = 1;
   k_o = 1;
   
   % Run the Decision Making Plan for Specified Number of Simulation
   for i = 1:numSimulations
      
      % Sample State from Gaussian Belief
      s = zeros(sys.n,1);
      for j = 1:(sys.n/3)
         ind2a = 3*j-2:3*j;
         try
            s(ind2a) = mvnrnd(mu_in(ind2a),sigma_in(ind2a,ind2a))';
         catch
            s(ind2a) = mu_in(ind2a);
         end
      end
      % Simulate One Branch via MCTS-DPW
      n_cur = 1;
      d = d_desired;
      [~] = simulate(s, d, n_cur, a_prev, o_prev, sys);
%       % Display Graph
%       figure(3); plot(g);
%       pause(0.1);
      
   end % end for all simulations
   
   % Return Next Action that Maximizes Q(bao)
   r_cur = -1E6;
   a_cur = 0;
   for i = 1:size(C{1},2)
      n_cur_a = C{1}(i);
      % Compute Mean Potential Observation Value from Action a
      r_sum = 0;
      for j = 1:size(C{n_cur_a},2)
         n_cur_o = C{n_cur_a}(j);
         r_sum = r_sum + Q{n_cur_o};
      end
      r_sum  = 1/(size(C{n_cur_a},2))*r_sum;
      % Check for Highest Value Action
      if ( r_sum>r_cur )
         r_cur = r_sum;
         a_cur = n_cur_a;
      end
   end
   
   % Return Final Action
   a_output = E{a_cur};
   
end

