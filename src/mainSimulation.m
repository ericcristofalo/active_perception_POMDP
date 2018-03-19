%--------------------------------------------------------------------------
%
% File Name:      mainSimulation_v1.m
% Date Created:   2016/01/26
% Date Modified:  2018/03/16
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    2016 AURO Simulation with EKF and UKF Estimation
%                 including quadrotor dynamics
%                 
%                 Transformation Notation: 
%                 Rotation matrices are defined as R_ab in this work.
%                 This equates to the tranformation of frame a to frame b.
%                 Points in each frame are related in the following manner:
%                    p_a = R_ab*p_b
%
%                 Structures:
%                 rob:     robot properties
%                 cam:     camera properties
%                 trans:   transformations
%                 stl:     imported stl properties
%                 obj:     scene object point coordinates
%                 p:       post-processing
%                 ctrl:    control properties
%
%
%--------------------------------------------------------------------------


%% Simulation Timing Initialization

% Add Paths
addpath('/Users/ericcristofalo/Dropbox/PhD/Research/2017_Active_Estimation/3D_Models');

% Check for Simulation Trials Script
if (~exist('monteCarloBool','var')) && (~exist('campBool','var'))
   clean('Figures',0);
   monteCarloBool = 0;
   compBool = 0;
elseif monteCarloBool==1
   % Do Nothing
elseif compBool==1
   % Do Nothing
else
   clean('Figures',0);
   monteCarloBool = 0;
   compBool = 0;
end

% Simulation Parameters
s.t0 = 0; % seconds
s.tf = 10*60; % seconds

% Simulation Parameters
s.dt = 0.005; % simulation period (seconds)
s.tSpan = s.t0:s.dt:s.tf; % seconds
s.tSpanLength = length(s.tSpan);
s.tSpanInd = 1:s.tSpanLength;
tInd = 1;

% Camera Simulation Parameters
cam.dt = 0.2;
cam.ind = 0;
cam.val = cam.dt/s.dt;
cam.tSpan = s.t0:cam.dt:s.tf;
cam.tSpanLength = length(cam.tSpan);
cam.tSpanInd = 1:cam.tSpanLength;
cam.tInd = 1;

% Control Condition
if compBool==0 && monteCarloBool==0
%    s.ctrlCond = 'Known_Velocities';
%    s.ctrlCond = 'Random_Walk';
%    s.ctrlCond = 'Greedy_EKF';
%    s.ctrlCond = 'Greedy_UKF';
%    s.ctrlCond = 'Rec_Horizon_EKF';
%    s.ctrlCond = 'Rec_Horizon_UKF';
   s.ctrlCond = 'MCTS_EKF';
%    s.ctrlCond = 'MCTS_UKF';
%    s.ctrlCond = 'Approximate_Circle';
%    s.ctrlCond = 'dSigma_dP_EKF';
%    s.ctrlCond = 'dSigma_dP_UKF';
end

% Estimation Condition
if compBool==0 && monteCarloBool==0
   % Choose from: EKF, UKF, LTVKF
   s.estCond = {'EKF'};
%    s.estCond = {'EKF','UKF','LTVKF'};
end

if compBool==0 && monteCarloBool==0
   s.filterCtrl = 'EKF';
end

% Number of Desired Object Points and Features to Track
% stl.filename = 'roman_colosseum';
stl.filename = 'random';
stl.filepath = [stl.filename,'.stl']; % stl file path
if compBool==0
   obj.n = 25; % number of feature points per image
end
obj.sample = 25; % number of points making the 3D model 
                    % 0 => all available points
obj.samePoint = 2; % 0 => chose random points
                   % 1 => same specified point
                   % 2 => origin + obj.n random points

% Simulation Noise Parameters
est.dynamicsNoise = 1;  % enable simulation noise on the robot motion
est.cameraNoise = 1;    % enable simulation noise on the camera measurements

% Estimation Parameters
est.n = obj.n; % number of total points to keep track of
               % 0 => all available points

est.explore = 0; % 0 => estimate (predict) until all features are lost
                 % 1 => estimate (predict) until all features are lost, select new set and continue
                 % 2 => estimate (predict) until one feature is lost, replace feature and continue

% Plotting Parameters
p.plotting = 1; % enable plotting while simulation is in progress
p.plotCovEllipsoids = 0;   % enable plotting of covariance ellipsoids
                           % a costly operations for many features
p.plotEKFellipsoidsOverride=0;
p.movie = 0;
% Plotting Period
p.plotInd = 0;
% p.plotVal = cam.val;
p.plotVal = 1000;
p.plotFinalFigures = 0;
p.simulationError = 0;


%% Simulation Environment Initialization

%--------------------------------------------------------------------------
% Complete Pose Initialization
%--------------------------------------------------------------------------

% Random Initial Velocity For Robot and Control
if compBool==0
%    ctrl.vStart = [rand(2,1)-0.5;0];
%    ctrl.vStart = ctrl.vStart/norm(ctrl.vStart)*0.09;
%    ctrl.vStart = [0.0048;-0.0999;0];
   ctrl.vStart = [0.0;0;0];
else
  ctrl.vStart = [obj.vStart(:,simInd);0];
end

% Initialize Robot Pose
rob.pose = zeros(6,s.tSpanLength);
rob.v = zeros(6,s.tSpanLength);
% rob.v(1:3,1) = [0;0.1;0];
rob.v(1:3,1) = ctrl.vStart;
rob.a = zeros(6,s.tSpanLength);

% Possible Robot Inital Conditions
p.poses = [... % [x, y, z, phi, theta, psi] in meters and degrees
   0,0,-10,0,0,0;
];
p.curPose = 1;
rob.pose(:,tInd) = ...
   [p.poses(p.curPose,1:3)';deg2rad(p.poses(p.curPose,4:6))'];

% Monte Carlo Random Initial Conditions
if monteCarloBool==1
   rob.pose(:,tInd) = ...
      [0;0;-10;deg2rad(p.poses(p.curPose,4:6))'];
   poseVol = [10;10;5];
   rob.pose(1:3,1) = rob.pose(1:3,1) + diag(poseVol)*rand(3,1)-0.5*poseVol;
end
% Comparison Trials Initial Conditions
if compBool==1
   rob.pose(1:3,1) = obj.robInit(:,simInd);
end
disp('Initial robot pose:');
disp(rob.pose(:,1));
disp('Initial robot velocity:');
disp(ctrl.vStart);

% Pixel Coordinates
% cam.imSize = [480, 640]; % [height, width]
cam.imSize = [1000,1000]; % [height, width]
cam.K = [500   0     cam.imSize(2)/2;
         0     500   cam.imSize(1)/2;
         0     0     1             ];
cam.K_inv = inv(cam.K);
cam.K_cp = [cam.K_inv,zeros(3,1);zeros(1,3),1];
cam.plotSize = [0 cam.imSize(2) 0 cam.imSize(1)]; % [0,width,0,height]
cam.plotInd = 0;

% Robot Frame wrt World Frame
trans.R_wr = euler2rot(rob.pose(4:6,tInd));

% Camera Frame wrt Robot Frame
cam.offset = [000.0000;    % meters
              000.0000;    % meters
              000.0000;    % meters
              000.0000;    % degrees
              000.0000;    % degrees
              000.0000];	% degrees
cam.offset(4:6,tInd) = deg2rad(cam.offset(4:6,1));
trans.R_rc = euler2rot(cam.offset(4:6,1));
trans.R_cr = inv(trans.R_rc);
cam.pose(1:3,tInd) = rob.pose(1:3,tInd)+trans.R_wr*(cam.offset(1:3,1));

% Camera Frame wrt World Frame
trans.R_wc = trans.R_wr*trans.R_rc; % camera points to robot, then robot to world points
trans.R_cw = inv(trans.R_wc);
cam.pose(4:6,tInd) = rot2euler(trans.R_wc);
trans.camTrans = [trans.R_cw,-trans.R_cw*cam.pose(1:3,tInd); ...
   0,0,0,1]; % using inverse of camera to world (world to camera transformation)

% Virtual Object Scene Initialization
initObject;


%--------------------------------------------------------------------------
% Initial Conditions Plot
%--------------------------------------------------------------------------

% Color Initialization
p.arrowLength = 1;
% obj.color.whitebg = [0.96 0.98 1]; % off-white bluish background
obj.color.whitebg = [1 1 1]; % white background
% obj.color.groundPlane = [0.85 0.90 1]; % light blue
obj.color.groundPlane = [0.55 0.77 0.55]; % grass green
obj.color.face = [0.8,0.7,0.5]; % stone brown
obj.color.edge = obj.color.face-0.25*ones(1,3);
% obj.color.pts = [0.2,0.2,0.3]; % dark brown
obj.color.pts = obj.color.edge;
obj.color.trackPts = [1,0.2,0];
p.trajColor = [1,0.5 ,0];
% Plotting Indices
p.plotL = 10.0;
% Plotting Volume
p.plotVol = [-p.plotL,p.plotL,...
             -p.plotL,p.plotL,...
             -p.plotL*2,0];

% Plot Environment Initial Conditions
if compBool==0 && monteCarloBool==0
   plotEnvironment;
end


%% 3D Active Estimation Simulation

%--------------------------------------------------------------------------
% Control Initialzation
%--------------------------------------------------------------------------

% Initialize Control Parameters
ctrl.nu = zeros(3,cam.tSpanLength);
% ctrl.nu(:,1) = [0;0.1;0];
ctrl.nu(:,1) = ctrl.vStart;
ctrl.rob.v_des(:,1) = ctrl.nu(:,1);
ctrl.nTemp = obj.n;

% Mean Squared Estimation Error
p.error = zeros(s.tSpanLength,1);
p.desiredError = 0.1; % desired error of 1 mm

% Plotting Initialization
p.lostFeature = 0;
p.simulationError = 0;

% Random Walk
est.randGain = 10;
est.randTime = 1;
est.randCur = 0;

% % Simulation Noise for Nice AURO Results
% est.dynCov = 0.025; % (m/s)^2
% est.rotCov = 0.0001; % (rad/s)^2
% est.camCov = 1;   % pixel^2
% est.q = est.dynCov/500;
% est.r = est.camCov*20;
% Simulation Noise
est.dynCov = 0.03; % (m/s)^2
est.rotCov = 0.001; % (rad/s)^2
est.camCov = 1;   % pixel^2
est.q = est.dynCov/100; % EKF
est.r = est.camCov*10;
% est.q = est.dynCov/100; % UKF
% est.r = est.camCov*10;
% est.q = 0.01; % LTVKF
% est.r = 0.01;

% Simulation Noise
if ~isfield(est,'dynCov')
   est.dynCov = est.q;
   est.rotCov = 0;
end
if ~isfield(est,'camCov')
   est.camCov = est.r;
end
est.m_p = zeros(2*obj.n,cam.tSpanLength); % measurement
est.m_v = zeros(6,s.tSpanLength); % dynamics

% Estimation Initialization
estimation;

% Controller Gains
ctrl.k_des = 0.09;
ctrl.Gamma = ctrl.k_des*[1 0 0;
                         0 1 0;
                         0 0 0];
ctrl.epsilon = 0.005;
ctrl.ekfRGain = est.r; % set to est.r for no effect
ctrl.ukfRGain = est.r; % set to est.r for no effect

% Quadrotor Dynamic Control Initialization
% Model
rob.m = 0.9; % kg
rob.J = ... % kg*m^2
   [4.8077196e+03/1000000, -1.7793851e-01/1000000, 8.0944927e+01/1000000;
    -1.7793851e-01/1000000, 4.8175937e+03/1000000, -2.3586788e+00/1000000;
    8.0944927e+01/1000000, -2.3586788e+00/1000000, 8.2709448e+03/1000000];
rob.J_inv = inv(rob.J);
rob.gravity =[0;0;9.81]; % m/s^2
rob.Omega = skewSymMat(rob.v(4:6,1));
% Control Inputs
ctrl.rob.v_des = zeros(3,s.tSpanLength);
ctrl.rob.vErr = zeros(3,1);
ctrl.rob.aErr = zeros(3,1);
ctrl.rob.yaw_des(1,1) = rob.pose(6);
ctrl.rob.yawCond = 0;
ctrl.rob.z_des = rob.pose(3,1);
ctrl.rob.alt = ctrl.rob.z_des;
ctrl.rob.att.euler_des = zeros(3,s.tSpanLength);

% Initialize Movie
M = [];
if p.movie==1
   figure(1); clf;
   pos = get(1,'Position');
   set(1,'Position',[pos(1:2),1067,748/2]);
   % M_length = s.tspanlength;
   % clear M;
   % M(M_length) = struct('cdata',[],'colormap',[]);
   % movieInd = 1;
   % Movie Name
   curTime = clock;
   curTime = [num2str(curTime(1)),sprintf('%02.0f',curTime(2)),...
      sprintf('%02.0f',curTime(3)),'_',sprintf('%02.0f',curTime(4)),...
      sprintf('%02.0f',curTime(5)),sprintf('%02.0f',curTime(6))];
   name = [pwd,'/Figures/',curTime,'.avi'];
   videoWriterObj = VideoWriter(name);
   open(videoWriterObj);
end


%--------------------------------------------------------------------------
% Run Simulation
%--------------------------------------------------------------------------

% Plot Estimates
% plotEstimatedPose; pause(0.01);

% Timer
tic
p.simTime = 0;
p.addFeatureSet = 0;

for tInd = 2:s.tSpanLength
   
   % Set Time
   t = s.tSpan(tInd);
   
   % Comparison and MC mainSimulation_v1 Loop Break
   if (compBool==1||monteCarloBool==1) && p.simulationError==1
      break;
   end
   
   % Simulate New Position Update
%    simCameraModel;
   simQuadrotorModel;
   
   % Run Active Vision Algorithm
   cam.ind = cam.ind+1;
   if (cam.ind==cam.val) || (cam.ind==2 && cam.tInd==1)
      cam.ind = 0;
      cam.tInd = cam.tInd+1;
      acquireFeatures;
      activePerceptionControl;
   end
   
   % Run Filter(s)
   estimation;
   
   % Plot Every p.plotVal Frames
   p.plotInd = p.plotInd+1;
   if p.plotInd==p.plotVal && p.plotting==1
      disp([num2str(tInd-1),' of ',num2str(s.tSpanLength-1),' completed']);
      plotEnvironment;
      p.plotInd = 0;
      % Save Figure to Movie
      if p.movie==1
         %    M(movieInd).cdata = getframe(gcf);
         %    movieInd = movieInd+1;
         frame = getframe(gcf);
         writeVideo(videoWriterObj,frame);
      end
      pause(0.1);
   end
   
end
p.simTime = toc;

if p.movie==1
   close(videoWriterObj);
end

% Final Plotting
if ( compBool==0 && monteCarloBool==0 )
   postProcessing;
end

return;


%% SAVE SIMULATION
% Save Simulation
filename = ['/Users/ericcristofalo/Dropbox/BU/Research/2015_Active_Estimation/3D_Depth_Estimation/Simulations/1_Feature_Comparison',...
%    '/active.mat'];
   '/circle.mat'];
save(filename,'rob','cam','est','ctrl','post','trans','s','obj','tInd');


%% SAVE FIGURES

return
FigureRange = [2];
% figureHandleRange = [];
% figureNames = {'20150225_1','20150225_2','20150225_3','20150225_4','20150225_5'};
extension = 'pdf';
saveFigures('FigureRange',FigureRange,'extension',extension);


%% SAVE WORKSPACE VARIABLES

return
data.points = obj.points;
data.pixelTrackPts = p.pixelTrackPts;
save([pwd,'/Data/data.mat'],'data')

