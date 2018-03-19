%--------------------------------------------------------------------------
%
% File Name:      simTrials.m
% Date Created:   2016/09/08
% Date Modified:  2016/09/26
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Script to run simulation trials of mainSimulation_v1
%
% Inputs:         numCtrls: row vector of indices of desired controls
%                    1: Random_Walk
%                    2: Greedy_'filterControl'
%                    3: dSigma/dP_'filterControl'
%                    4: Approximate_Circle
%                    5: Known_Velocities
%
%--------------------------------------------------------------------------

clean('Figures',0);

%% Parameter Selection

% Initialize Simulation Parameters
monteCarloBool = 0; % set estimation parameters in mainSimulation_v1
compBool = 1;

numSims = 25; % number of different simulations to run

s.estCond = {'EKF'};    % desired filter(s) for estimation
s.filterCtrl = 'EKF';   % desired filter for control

% Monte Carlo Simulation Inputs
if monteCarloBool==1
%    s.ctrlCond = 'Known_Velocities';
%    s.ctrlCond = 'Random_Walk';
%    s.ctrlCond = 'Greedy_EKF';
%    s.ctrlCond = 'Greedy_UKF';
%    s.ctrlCond = 'Rec_Horizon_EKF';
%    s.ctrlCond = 'Rec_Horizon_UKF';
%    s.ctrlCond = 'MCTS_EKF';
%    s.ctrlCond = 'MCTS_UKF';
%    s.ctrlCond = 'Approximate_Circle';
%    s.ctrlCond = 'dSigma_dP_EKF';
%    s.ctrlCond = 'dSigma_dP_UKF';
end

% Comparison Simulation Inputs
if compBool==1
%    numCtrls = [1,2,3,4,5];	% number of different control options
%    numCtrls = 3;
   numCtrls = [1,4];	% number of different control options
end


%% Initialization

% Check for Error
if monteCarloBool==compBool
   error('Error: Must run one trial only. Monte Carlo or Comparison. ');
end

% Generate New Simulation Folder
curTime = clock;
curTime = [num2str(curTime(1)),sprintf('%02.0f',curTime(2)),...
           sprintf('%02.0f',curTime(3)),'_',sprintf('%02.0f',curTime(4)),...
           sprintf('%02.0f',curTime(5)),sprintf('%02.0f',curTime(6))];
% Monte Carlo Initialization
if monteCarloBool==1
   folder = 'Monte_Carlo_Trials';
   numCtrls = 1;
end
% Comparison Initialization
if compBool==1
   folder = 'Comparison_Trials';
   % Generate Static Object for All Trials
   obj.n = 25;
   obj.sample = obj.n;
   obj.randV = [1.5,1.5,-3];
   obj.v = [obj.randV(1)*2,0,0;0,obj.randV(2)*2,0;0,0,obj.randV(3)]*rand(3,obj.sample-1)-...
      repmat([obj.randV(1);obj.randV(2);0],1,obj.sample-1);
   obj.v = [[[0;0;0],obj.v];linspace(1,obj.sample,obj.sample)];
   
   % Generate Random Initial Conditions For Each Simulation
   poseVol = [5;5;5];
   obj.robInit =  repmat([0;0;-10],1,numSims)+diag(poseVol)*rand(3,numSims)-0.5*repmat(poseVol,1,numSims);
   obj.Pz_0 = 49*rand(numSims,1)*ones(1,obj.n)+1; % random guess
   
%    % Static Initial Conditions % COMMENT FOR MULTISIM COMPARISON
%    obj.Pz_0 = 15*ones(1,obj.n);
%    obj.robInit =  repmat([0;0;-10],1,numSims);
   
   % Initial Velocity
   obj.vStart = zeros(2,numSims);
%    ctrl.k_des = 0.1; % MATCH IN MAINSIM FOR INITIAL CONDITIONS
%    obj.vStart = 2*ctrl.k_des*rand(2,numSims)-ctrl.k_des;
%    for i = 1:numSims
%       if norm(obj.vStart(:,i))>ctrl.k_des;
%          obj.vStart(:,i) = ctrl.k_des*obj.vStart(:,i)./norm(obj.vStart(:,i));
%       end
%    end

   % READ PREVIOUS POINTS FROM OTHER TRIAL
   load('/Users/ericcristofalo/Dropbox/PhD/Research/2018_Active_Estimation/2018_TCST/Simulation/Comparison_Trials/EKF_01_25trials_25feat/obj.mat');
   
end
folderPath = [pwd,'/',folder,'/',curTime];
% Create Specific Figure Filepath if None Exists in Current Directory
if exist(folderPath,'file')~=7
   mkdir(folderPath);
   % Make Seperate Control Conditions Folders
   if compBool==1
      controlConditions = {...
         'Random_Walk',...
         ['Greedy_',s.filterCtrl],...
         ['Rec_Horizon_',s.filterCtrl],...
         ['MCTS_',s.filterCtrl],...
         ['dSigma_dP_',s.filterCtrl],...
         'Approximate_Circle',...
         'Known_Velocities'};
      for ctrlInd = numCtrls
         mkdir([folderPath,'/',controlConditions{ctrlInd}]);
      end
   end
   % Save Information File
   save([folderPath,'/','information.mat'],'s','numCtrls','numSims','folderPath');
end


%% Run Simulation Trials

for simInd = 1:numSims
   
   % Clear Simulation
   disp(['Simulation: ',num2str(simInd)]);
   
   % Save Simulation Data
   if monteCarloBool==1 
      
      % Run Monte Carlo Trial
      clear rob cam est ctrl p trans ekf ukf;
      mainSimulation;
      filename = [folderPath,'/sim_',num2str(simInd),'.mat'];
      save(filename,'rob','cam','est','ctrl','p','trans','s','obj','tInd');
      
      % Save Data for Future
      px = reshape(mean(p.pixelTrackPts(1,:,1:cam.tInd),2),1,cam.tInd);
      py = reshape(mean(p.pixelTrackPts(2,:,1:cam.tInd),2),1,cam.tInd);
      diff = cam.tSpanLength-size(px,2);
      pxSave(simInd,:) = [px,-1*ones(diff,1)'];
      pySave(simInd,:) = [py,-1*ones(diff,1)'];
      
   else
      
      % Control Comparison Simulation
      for ctrlInd = numCtrls
         
         % Select Control Option
         s.ctrlCond = controlConditions{ctrlInd};
         disp(s.ctrlCond);
         
         % Run Control Simulation
         clear rob cam est ctrl p trans ekf ukf;
         mainSimulation;
         filename = [folderPath,'/',s.ctrlCond,'/sim_',num2str(simInd),'.mat'];
         save(filename,'rob','cam','est','ctrl','p','trans','s','obj','tInd');
         
      end
      
   end
   
end

% Finish Saving Data
if monteCarloBool==1
   % Save Monte Carlo Run
   filename = [folderPath,'/pixelData.mat'];
   save(filename,'pxSave','pySave');
end
if compBool==1
   
end
