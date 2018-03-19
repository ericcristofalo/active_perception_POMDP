%--------------------------------------------------------------------------
%
% File Name:      activePerceptionControl.m
% Date Created:   2016/09/02
% Date Modified:  2018/01/20
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    AURO Active Perception Control Computation
%
%--------------------------------------------------------------------------


%% Active Perception Control

%--------------------------------------------------------------------------
if strcmp(s.ctrlCond,'Known_Velocities')
  tic
  %    % Oscillating Translational Velocity Inputs (Outward Spiral)
  %    ctrl.nu(1,cam.tInd) = ctrl.k_des*sin(1/5*t);%*t/40;
  %    ctrl.nu(2,cam.tInd) = ctrl.k_des*cos(1/5*t);%*t/50;
  %    ctrl.nu(3,cam.tInd) = ctrl.k_des*sin(1*t);
  %    ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd)./norm(ctrl.nu(:,cam.tInd));
  
  %    % Ocillating Circles
  %    ctrl.nu(1,cam.tInd) = ctrl.k_des*sin(1*t)*2;
  %    ctrl.nu(2,cam.tInd) = ctrl.k_des*cos(1*t)*2;
  %    ctrl.nu(3,cam.tInd) = ctrl.k_des*sin(1*t);
  
  % Test
  ctrl.nu(:,cam.tInd) = ctrl.k_des*[1;0;0];
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'Random_Walk')
  tic
  
%   % Random Walk
%   est.randCur = est.randCur+1;
%   if est.randCur==est.randTime
%     %       if cam.tInd<est.randGain
%     %          est.randTime = 1;
%     %       else
%     est.randTime = ceil(rand*est.randGain);
%     %       end
%     est.randCur = 0;
%     uTemp = 2*rand(2,1)-1;
%     ctrl.nu(1:2,cam.tInd) = ctrl.k_des*uTemp./(norm(uTemp)+ctrl.epsilon);
%   else
%     ctrl.nu(1:2,cam.tInd) = ctrl.nu(1:2,cam.tInd-1);
%   end
  
  uTemp = 2*rand(2,1)-1;
  ctrl.nu(1:2,cam.tInd) = ctrl.k_des*uTemp./(norm(uTemp)+ctrl.epsilon);
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'Greedy_EKF')
  tic
  
  % Size of Update
  %    ctrl.ID = est.updateID;
  ctrl.ID = est.predictID;
  n = size(ctrl.ID,2);
  
  % Populate Large Matrices
  H = zeros(2*n,3*n);
  R = zeros(2*n,2*n);
  Sigma = zeros(3*n,3*n);
  Kt = zeros(2*n,3*n);
  P = zeros(3*n,1);
  for i = 1:n
    % Indices
    ind1 = 2*i-1:2*i;
    ind2 = 3*i-2:3*i;
    uInd1 = 2*ctrl.ID(i)-1:2*ctrl.ID(i);
    uInd2 = 3*ctrl.ID(i)-2:3*ctrl.ID(i);
    % Estimates
    P(uInd2,1) = est.P_ekf(uInd2,cam.tInd-1);
    Sigma(ind2,ind2) = est.Sigma_ekf(uInd2,uInd2)+est.Q(uInd2,uInd2);
    Kt(ind1,ind2) = est.K_cam;
    R(ind1,ind1) = est.R(uInd1,uInd1);
  end
  
  % Compute Cost and Determine Minimizing Argument
  numCtrlOpts = 200;
  angleOpts = linspace(0,2*pi,numCtrlOpts);
  uOpts = [sin(angleOpts);cos(angleOpts);ones(1,numCtrlOpts)];
  JOpts = zeros(1,numCtrlOpts);
  for i = 1:numCtrlOpts
    % Candidate Control Input
    P_ = P-repmat(uOpts(:,i),n,1)*s.dt;
    for j = 1:n
      % Indices
      ind1 = 2*j-1:2*j;
      ind2 = 3*j-2:3*j;
      H(ind1,ind2) = est.K_cam*...
        [1/P_(3*j), 0, -P_(3*j-2)/P_(3*j)^2;
        0, 1/P_(3*j), -P_(3*j-1)/P_(3*j)^2;
        0, 0, 0];
    end
    % Cost Function Comparison
    F = H*Sigma*H'+R;
    JOpts(1,i) = trace(-Sigma*H'/F*H*Sigma);
  end
  [val,ind] = min(JOpts);
  ctrl.uTemp = uOpts(:,ind);
  
  % Normalized Control Input
  ctrl.nu(:,cam.tInd) = ctrl.Gamma*ctrl.uTemp/(norm(ctrl.uTemp)+ctrl.epsilon);
  
  %    % Drive straight at first
  %    if cam.tInd=5
  % %       ctrl.nu(:,cam.tInd) = ctrl.k_des*ctrl.vStart;
  %       ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd-1);
  %    end
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'Greedy_UKF')
  tic
  
  % Size of Update
  %    ctrl.ID = est.updateID;
  ctrl.ID = est.predictID;
  n = size(ctrl.ID,2);
  
  % Previous Data
  P = est.P_ukf(:,cam.tInd-1);
  est.Sigma_ukf_ = est.Sigma_ukf+est.Q;
  
  % Generate Sigma Vector
  ukf.Cov_a = est.Sigma_ukf_;
  %    ukf.Cov_a = [ukf.Cov_a];%,zeros(3*est.n,2*est.n);zeros(2*est.n,3*est.n),R];
  % UT Transform (Wan et al. 2000 (Wikipedia))
  ukf.CovSquare = chol((ukf.L+ukf.lambda)*ukf.Cov_a)';
  %    % UT Transform (Uhlmann et al. 1997)
  %    ukf.CovSquare = chol((ukf.L+ukf.kappa)*ukf.Cov_a)';
  %    ukf.CovSquare = chol(ukf.lambda*ukf.Cov_a)';
  
  % Compute Cost and Determine Minimizing Argument
  numCtrlOpts = 20;
  angleOpts = linspace(0,2*pi,numCtrlOpts);
  uOpts = [sin(angleOpts);cos(angleOpts);ones(1,numCtrlOpts)];
  JOpts = zeros(1,numCtrlOpts);
  for u = 1:numCtrlOpts
    % Candidate Control Input
    P_ = P-repmat(uOpts(:,u),n,1)*s.dt;
    % Compute Sigma Vectors
    ChiTemp = repmat(P_,1,ukf.n_a)+[zeros(3*est.n,1),ukf.CovSquare,-ukf.CovSquare];
    % Propagated Measurement
    ukf.ySamples = zeros(2*est.n,ukf.n_a);
    for i = 1:est.n
      ind1 = 2*i-1:2*i;
      ind2 = 3*i-2:3*i;
      for j = 1:(ukf.n_a)
        ukf.ySamples(ind1,j) = 1/(ChiTemp(3*i,j))*est.K_cam*ChiTemp(ind2,j);%+est.chi(4:5,i,tInd);
      end
    end
    ukf.yHat(:,cam.tInd) = ukf.ySamples*ukf.weights(1,:)';
    
    % Covariance Matrices
    diffP = ukf.chi(:,:,cam.tInd)-repmat(P_,1,ukf.n_a);
    diffy = ukf.ySamples-repmat(ukf.yHat(:,cam.tInd),1,ukf.n_a);
    Cov_yy = diffy*diag(ukf.weights(2,:))*diffy'+est.R;
    Cov_Py = diffP*diag(ukf.weights(2,:))*diffy';
    
    % Final Update (Uhlmann et al. 1997)
    K = Cov_Py/Cov_yy;
    JOpts(1,u) = trace(-K*Cov_yy*K'); % covariance update
  end
  [val,ind] = min(JOpts);
  ctrl.uTemp = uOpts(:,ind);
  
  % Normalized Control Input
  ctrl.nu(:,cam.tInd) = ctrl.Gamma*ctrl.uTemp/(norm(ctrl.uTemp)+ctrl.epsilon);
  
  %    % Drive straight at first
  %    if cam.tInd<5
  % %       ctrl.nu(:,cam.tInd) = ctrl.k_des*ctrl.vStart;
  %       ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd-1);
  %    end
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'Rec_Horizon_EKF')
  tic
  
  % Size of Update
  %    ctrl.ID = est.updateID;
  ctrl.ID = est.predictID;
  n = size(ctrl.ID,2);
  
  % Control Parameters
  numCtrlOpts = 12;
  angleOpts = linspace(0,2*pi,numCtrlOpts);
  uOpts = [sin(angleOpts);cos(angleOpts);ones(1,numCtrlOpts)];
  numHorizon = 3;
  costMat = zeros(numCtrlOpts^numHorizon,numHorizon);
  stateMat = cell(size(costMat));
  sigmaMat = cell(size(costMat));
  gamma = 0.9;
  
  % Initial Conditions
  H = zeros(2*n,3*n);
  R = zeros(2*n,2*n);
  Sigma_0 = zeros(3*n,3*n);
  Kt = zeros(2*n,3*n);
  P_0 = zeros(3*n,1);
  for i = 1:n
    % Indices
    ind1 = 2*i-1:2*i;
    ind2 = 3*i-2:3*i;
    uInd1 = 2*ctrl.ID(i)-1:2*ctrl.ID(i);
    uInd2 = 3*ctrl.ID(i)-2:3*ctrl.ID(i);
    % Estimates
    P_0(uInd2,1) = est.P_ekf(uInd2,cam.tInd-1);
    Sigma_0(ind2,ind2) = est.Sigma_ekf(uInd2,uInd2)+est.Q(uInd2,uInd2);
  end
  
  % Iterate Through Horizons
  for hInd = 1:numHorizon
    % Iterate Through Control Options
    ind_ = 1;
    uInd = 1;
    for uCur = 1:numCtrlOpts^hInd % uCur is the current storing index
      
      % Select Appropriate Previous State from Control Tree
      if hInd==1
        P = P_0;
        Sigma = Sigma_0;
      else
        P = stateMat{ind_,hInd-1};
        Sigma = sigmaMat{ind_,hInd-1};
      end
      
      % Propogate State Prediction with Current Control
      P_ = P-repmat(uOpts(:,uInd),n,1)*s.dt;
      expOmegaFull = kron(eye(est.nTemp),eye(3,3));
      Sigma_ = expOmegaFull*Sigma*expOmegaFull'+est.Q;
      
      % Simulate EKF Update
      Hcur = zeros(2*n,3*n);
      hCur = zeros(2*n,1);
      y_simulated = zeros(2*n,1);
      ekf.G = zeros(2*n,2*n);
      ekf.K = zeros(3*n,2*n);
      for i = 1:n
         ind1a = 2*i-1:2*i;
         ind2a = 3*i-2:3*i;
         P = P_(ind2a,1);
         Hcur(ind1a,ind2a) = est.K_cam*...
            [1/P(3), 0, -P(1)/P(3)^2;
            0, 1/P(3), -P(2)/P(3)^2;
            0, 0, 0];
         hCur(ind1a,1) = 1/P(3)*est.K_cam*P;
%          y_simulated(ind1a,1) = 1/P(3)*est.K_cam*P + normrnd(0,est.camCov,[2,1]);
      end
      % Kalman Gain Update
      ekf.G = (Hcur*Sigma_*Hcur' + est.R);
      ekf.K = Sigma_*Hcur'/ekf.G;
%       % State Estimate Update
%       ekf.correction = ekf.K*(y-hCur);
%       Pcur = Pcur_(uInd2,1)+ekf.K*(y-hCur);
      % Prediction Covariance Update
      Sigma = Sigma_ - ekf.K*Hcur*Sigma_;
      
      % Save New States and Covariances
      stateMat{uCur,hInd} = P_;
      sigmaMat{uCur,hInd} = Sigma;
      if hInd>1
        costMat(uCur,hInd) = costMat(ind_,hInd-1) + gamma^(hInd-1)*trace(Sigma);
      else
        costMat(uCur,hInd) = trace(Sigma);
      end
      
      % Update Previous Storing Index
      if mod(uCur,numCtrlOpts)==0
        ind_ = ind_+1;
        uInd = 1;
      else
        uInd = uInd+1;
      end
      
    end
  end
  
  % Find Branch of Least Cost
  [val,ind] = min(costMat(:,end));
  % Previous Control
  for hInd = 1:(numHorizon-1)
    ind = (ind-mod(ind,numCtrlOpts))/numCtrlOpts + 1;
  end
  ctrl.uTemp = uOpts(:,ind);
  
  % Normalized Control Input
  ctrl.nu(:,cam.tInd) = ctrl.Gamma*ctrl.uTemp/(norm(ctrl.uTemp)+ctrl.epsilon);
  
  %    % Drive straight at first
  %    if cam.tInd=5
  % %       ctrl.nu(:,cam.tInd) = ctrl.k_des*ctrl.vStart;
  %       ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd-1);
  %    end
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'Rec_Horizon_UKF')
  tic
  
  % Size of Update
  %    ctrl.ID = est.updateID;
  ctrl.ID = est.predictID;
  n = size(ctrl.ID,2);
  
  % Control Parameters
  numCtrlOpts = 12;
  angleOpts = linspace(0,2*pi,numCtrlOpts);
  uOpts = [sin(angleOpts);cos(angleOpts);ones(1,numCtrlOpts)];
  numHorizon = 2;
  costMat = zeros(numCtrlOpts^numHorizon,numHorizon);
  stateMat = cell(size(costMat));
  sigmaMat = cell(size(costMat));
  
% % %   % Previous Data
% % %   P = est.P_ukf(:,cam.tInd-1);
% % %   est.Sigma_ukf_ = est.Sigma_ukf+est.Q;
% % %   
% % %   % Generate Sigma Vector
% % %   ukf.Cov_a = est.Sigma_ukf_;
% % %   %    ukf.Cov_a = [ukf.Cov_a];%,zeros(3*est.n,2*est.n);zeros(2*est.n,3*est.n),R];
% % %   % UT Transform (Wan et al. 2000 (Wikipedia))
% % %   ukf.CovSquare = chol((ukf.L+ukf.lambda)*ukf.Cov_a)';
% % %   %    % UT Transform (Uhlmann et al. 1997)
% % %   %    ukf.CovSquare = chol((ukf.L+ukf.kappa)*ukf.Cov_a)';
% % %   %    ukf.CovSquare = chol(ukf.lambda*ukf.Cov_a)';

  % Initial Conditions
  H = zeros(2*n,3*n);
  R = zeros(2*n,2*n);
%   Sigma_0 = zeros(3*n,3*n);
  Kt = zeros(2*n,3*n);
  P_0 = zeros(3*n,1);
  Cov_a = est.Sigma_ukf_;
  CovSquare = chol((ukf.L+ukf.lambda)*ukf.Cov_a)';
  Sigma_0 = est.Sigma_ukf;
  for i = 1:n
    % Indices
    ind1 = 2*i-1:2*i;
    ind2 = 3*i-2:3*i;
    uInd1 = 2*ctrl.ID(i)-1:2*ctrl.ID(i);
    uInd2 = 3*ctrl.ID(i)-2:3*ctrl.ID(i);
    % Estimates
    P_0(uInd2,1) = est.P_ukf(uInd2,cam.tInd-1);
%     Sigma_0(ind2,ind2) = est.Sigma_ukf(uInd2,uInd2)+est.Q(uInd2,uInd2);
  end
  
  % Iterate Through Horizons
  for hInd = 1:numHorizon
    % Iterate Through Control Options
    ind_ = 1;
    uInd = 1;
    for uCur = 1:numCtrlOpts^hInd % uCur is the current storing index
      
      % Select Appropriate Previous State from Control Tree
      if hInd==1
        P = P_0;
        Sigma = Sigma_0;
      else
        P = stateMat{ind_,hInd-1};
        Sigma = sigmaMat{ind_,hInd-1};
      end

      % Propogate State Prediction with Current Control
      P_ = P-repmat(uOpts(:,uInd),n,1)*s.dt;
      expOmegaFull = kron(eye(est.nTemp),eye(3,3));
      Sigma_ = expOmegaFull*Sigma*expOmegaFull'+est.Q;
      
      % Compute Sigma Vectors
      Cov_a = Sigma_;
      CovSquare = chol((ukf.L+ukf.lambda)*ukf.Cov_a)';
      ChiTemp = repmat(P_,1,ukf.n_a)+[zeros(3*est.n,1),CovSquare,-CovSquare];
      % Propagated Measurement
      ySamples = zeros(2*est.n,ukf.n_a);
      for i = 1:est.n
        ind1 = 2*i-1:2*i;
        ind2 = 3*i-2:3*i;
        for j = 1:(ukf.n_a)
          ySamples(ind1,j) = 1/(ChiTemp(3*i,j))*est.K_cam*ChiTemp(ind2,j);%+est.chi(4:5,i,tInd);
        end
      end
      yHat(:,cam.tInd) = ySamples*ukf.weights(1,:)';
      % Covariance Matrices
      diffP = ukf.chi(:,:,cam.tInd)-repmat(P_,1,ukf.n_a);
      diffy = ySamples-repmat(yHat(:,cam.tInd),1,ukf.n_a);
      Cov_yy = diffy*diag(ukf.weights(2,:))*diffy'+est.R;
      Cov_Py = diffP*diag(ukf.weights(2,:))*diffy';
      % Final Update (Uhlmann et al. 1997)
      K = Cov_Py/Cov_yy;
%       Pcur = Pcur_ + updateMat*K*(y-ukf.yHat(:,cam.tInd));      % state update
      Sigma = Sigma_-K*Cov_yy*K'; % covariance update
      
      % Save New States and Covariances
      stateMat{uCur,hInd} = P_;
      sigmaMat{uCur,hInd} = Sigma;
      if hInd>1
        costMat(uCur,hInd) = costMat(ind_,hInd-1) + gamma^(hInd-1)*trace(Sigma);
      else
        costMat(uCur,hInd) = trace(Sigma);
      end
      
      % Update Previous Storing Index
      if mod(uCur,numCtrlOpts)==0
        ind_ = ind_+1;
        uInd = 1;
      else
        uInd = uInd+1;
      end
      
    end
  end

  % Find Branch of Least Cost
  [val,ind] = min(costMat(:,end));
  % Previous Control
  for hInd = 1:(numHorizon-1)
    ind = (ind-mod(ind,numCtrlOpts))/numCtrlOpts + 1;
  end
  if (ind>numCtrlOpts)
    ind = numCtrlOpts;
  end
  ctrl.uTemp = uOpts(:,ind);
  
  % Normalized Control Input
  ctrl.nu(:,cam.tInd) = ctrl.Gamma*ctrl.uTemp/(norm(ctrl.uTemp)+ctrl.epsilon);
  
  %    % Drive straight at first
  %    if cam.tInd=5
  % %       ctrl.nu(:,cam.tInd) = ctrl.k_des*ctrl.vStart;
  %       ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd-1);
  %    end
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'MCTS_EKF')
  tic
  
  % Size of Update
  %    ctrl.ID = est.updateID;
  ctrl.ID = est.predictID;
  n = size(ctrl.ID,2);
  
  % Initial Conditions
  H = zeros(2*n,3*n);
  R = zeros(2*n,2*n);
  Sigma_0 = zeros(3*n,3*n);
  Kt = zeros(2*n,3*n);
  P_0 = zeros(3*n,1);
  for i = 1:n
    % Indices
    ind1 = 2*i-1:2*i;
    ind2 = 3*i-2:3*i;
    uInd1 = 2*ctrl.ID(i)-1:2*ctrl.ID(i);
    uInd2 = 3*ctrl.ID(i)-2:3*ctrl.ID(i);
    % Estimates
    P_0(uInd2,1) = est.P_ekf(uInd2,cam.tInd-1);
    Sigma_0(ind2,ind2) = est.Sigma_ekf(uInd2,uInd2)+est.Q(uInd2,uInd2);
  end
  
   % Set Up MCTS-DPW Input
   sys.n = 3*n;
   sys.m = 2*n;
   sys.F = eye(sys.n);
   sys.B = repmat(eye(3),n,1);
   sys.Q = est.q*eye(sys.n);
   sys.H = est.K_cam;
   sys.R = est.r*eye(sys.m);
  
	% Monte Carlio Tree Search (MCTS) with Double Progressive Widening (DPW)
   d_desired = 10; % desired MCTS search depth
   a = mcts_dpw(P_0,Sigma_0,ctrl.nu(:,cam.tInd-1),est.y(:,cam.tInd-1),sys,d_desired);
   ctrl.uTemp = a;
  
  % Normalized Control Input
  ctrl.nu(:,cam.tInd) = ctrl.Gamma*ctrl.uTemp/(norm(ctrl.uTemp)+ctrl.epsilon);
  
  %    % Drive straight at first
  %    if cam.tInd=5
  % %       ctrl.nu(:,cam.tInd) = ctrl.k_des*ctrl.vStart;
  %       ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd-1);
  %    end
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'MCTS_UKF')
  tic
  
  
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'Approximate_Circle')
  tic
  
  ctrl.p = zeros(2,1);
  ctrl.P = zeros(3,1);
  for i = 1:est.n
    %       ind1 = 2*i-1:2*i;
    ind2 = 3*i-2:3*i;
    ctrl.p = p.pixelTrackPts(1:2,i,cam.tInd-1)+ctrl.p;
    if strcmp(s.filterCtrl,'EKF')
      ctrl.P = est.P_ekf(ind2,cam.tInd-1)+ctrl.P;
    elseif strcmp(s.filterCtrl,'UKF')
      ctrl.P = est.P_ukf(ind2,cam.tInd-1)+ctrl.P;
    elseif strcmp(s.filterCtrl,'LTVKF')
      ctrl.P = est.P_ltvkf(ind2,cam.tInd-1)+ctrl.P;
    end
  end
  %    ctrl.p = ctrl.p./norm(ctrl.p); % better for UKF comparison for some reason.
  %    ctrl.P = ctrl.P./norm(ctrl.P);
  ctrl.p = ctrl.p./est.n-[cam.K(1,3);cam.K(2,3)]; % original
  ctrl.P = ctrl.P./est.n;
  
  % AURO v1 Control
  rDes = 300;
  Kc = cam.K(1,1)*ctrl.k_des*10*cam.dt/ctrl.P(3);
  %    Kc = rDes;
  del = real(asin(Kc/(2*rDes)));
  yUnit = ctrl.p./norm(ctrl.p);
  v_d = rotMat(pi/2+del)*Kc*yUnit;
  uTemp = -(v_d + rDes*yUnit - ctrl.p);
  
  % Final Control Input
  ctrl.nu(1:2,cam.tInd) = ctrl.k_des*uTemp./(norm(uTemp)+ctrl.epsilon); % normalize control
  if sum(ctrl.p)==0
    ctrl.nu(1:2,cam.tInd) = ctrl.k_des*[0;-1];
  end
  
  %    % Drive straight at first
  %    if cam.tInd<5
  % %       ctrl.nu(:,cam.tInd) = ctrl.k_des*ctrl.vStart;
  %       ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd-1);
  %    end
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'dSigma_dP_EKF')
  tic
  
  % Size of Update
  ctrl.ID = est.updateID;
  %    ctrl.ID = est.predictID;
  n = size(ctrl.ID,2);
  
  % Populate Large Matrices
  H = zeros(2*n,3*n);
  R = ctrl.ekfRGain*eye(2*n,2*n);
  Sigma = zeros(3*n,3*n);
  Kt = zeros(2*n,3*n);
  for i = 1:n
    % Indices
    ind1 = 2*i-1:2*i;
    ind2 = 3*i-2:3*i;
    uInd1 = 2*ctrl.ID(i)-1:2*ctrl.ID(i);
    uInd2 = 3*ctrl.ID(i)-2:3*ctrl.ID(i);
    % Estimates
    P = est.P_ekf(uInd2,cam.tInd-1);
    %          Sigma(ind2,ind2) = est.Cov_(uInd2,uInd2,tInd-1);
    Sigma(ind2,ind2) = est.Sigma_ekf_(uInd2,uInd2);
    % Linearization
    H(ind1,ind2) = est.K_cam*...
      [1/P(3), 0, -P(1)/P(3)^2;
      0, 1/P(3), -P(2)/P(3)^2;
      0, 0, 0];
    Kt(ind1,ind2) = est.K_cam;
    %          R(ind1,ind1) = est.R(uInd1,uInd1);
  end
  
  
  % Compute Gradient for Each State (3n total states)
  gamma = zeros(3*n,1);
  for i = 1:n
    % Indices
    ind1 = 2*i-1:2*i;
    ind2 = 3*i-2:3*i;
    uInd1 = 2*ctrl.ID(i)-1:2*ctrl.ID(i);
    uInd2 = 3*ctrl.ID(i)-2:3*ctrl.ID(i);
    % Current Estimate
    P = est.P_ekf(uInd2,cam.tInd-1);
    % For each State in Current Estimate
    for j = 1:3
      % Compute Partial Derivates
      dHdP = zeros(3*n,3*n);
      if j==1
        dHdP(ind2,ind2) = [0, 0, -1/P(3)^2;
          0, 0, 0;
          0, 0, 0];
      elseif j==2
        dHdP(ind2,ind2) = [0, 0, 0;
          0, 0, -1/P(3)^2;
          0, 0, 0];
      elseif j==3
        dHdP(ind2,ind2) = [-1/P(3)^2, 0, 2*P(1)/P(3)^3;
          0, -1/P(3)^2, 2*P(2)/P(3)^3;
          0, 0, 0];
      end;
      % Compute Gradient For Current State
      dH = Kt*dHdP;
      F = H*Sigma*H'+R;
      dF = Kt*dHdP*Sigma*H'+H*Sigma*dHdP'*Kt';
      %          gamma(3*i-2+(j-1),1) = trace(-Sigma*...
      %          (dH'/F*H-H'/F*dF/F*H+H'/F*dH)*...
      %          Sigma);
      gamma(3*i-2+(j-1),1) = trace(-Sigma*...
        (2*dH'/F*H-H'/F*dF/F*H)*...
        Sigma);
    end
  end
  
  % Multiple Feature Normalized Sum
  gain = zeros(3,1);
  gSave = zeros(3,1);
  for i = 1:n
    ind2 = 3*i-2:3*i;
    uInd2 = 3*ctrl.ID(i)-2:3*ctrl.ID(i);
    P = est.P_ekf(uInd2,cam.tInd-1);
    g = gamma(ind2,1);
    gSave = gSave+g;
    gain = (est.expOmega-eye(3))*P+g+gain;
  end
  gain(3) = 0; % remove z-component
  ctrl.uTemp = gain./norm(gain);
  est.traceVector(1:3,cam.tInd) = gSave;
  
  % Normalized Control Input
  %    ctrl.nu(:,cam.tInd) = ctrl.uTemp;
  %    ctrl.nu(1:2,cam.tInd) = gain(1:2,1);
  ctrl.nu(:,cam.tInd) = ctrl.Gamma*ctrl.uTemp/(norm(ctrl.uTemp)+ctrl.epsilon);
  
  % Drive straight at first
  if cam.tInd>2 && cam.tInd<=4
    %       ctrl.nu(:,cam.tInd) = ctrl.vStart;
    ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd-1);
  end
  
  p.ctrlTime(1,cam.tInd) = toc;
  
  %--------------------------------------------------------------------------
elseif strcmp(s.ctrlCond,'dSigma_dP_UKF')
  
  tic
  
  % Handling Feature Loss: Size of Update
  ctrl.ID = est.updateID;
  %    ctrl.ID = est.predictID;
  n = size(ctrl.ID,2);
  %    n_a = 2*3*n_a+1;
  
  % Previous Estimates
  P = est.P_ukf(:,cam.tInd-1);
  chi = ukf.chi(:,:,cam.tInd-1);
  ySamples = ukf.ySamples;
  yHat = ukf.yHat(:,cam.tInd-1);
  
  % Pre-compute Covariances
  ukf.chiDiff = repmat(P,1,ukf.n_a)-chi;
  ukf.yDiff = repmat(yHat,1,ukf.n_a)-ySamples;
  % Sample Cross-Covariance
  Sig_xy = ukf.chiDiff*ukf.Wc*ukf.yDiff';
  % Sample Auto-Covariance
  R = ctrl.ukfRGain*eye(2*est.n,2*est.n);
  Sig_yy = ukf.yDiff*ukf.Wc*ukf.yDiff'+R;
  
  % Compute Gradient
  ukf.dhdx = zeros(2*est.n,ukf.n_a);
  ukf.dhdx_sum = zeros(2*est.n);
  ukf.dhdxDiff = zeros(2*est.n,ukf.n_a);
  dSig_xy = zeros(3*est.n,2*est.n);
  dSig_yy = zeros(2*est.n,2*est.n);
  gamma = zeros(3*est.n,1);
  for i = 1:est.n % for each feature
    ind1 = 2*i-1:2*i;
    ind2 = 3*i-2:3*i;
    for m = 1:3 % for each direction
      ukf.dhdx = zeros(2*est.n,ukf.n_a);
      for j = 1:ukf.n_a % for each sigma vector
        % Partial of Measurement Function
        if m==1
          ukf.dhdx(ind1,j) = est.K_cam*[1/chi(3*i,j);0;0];
        elseif m==2
          ukf.dhdx(ind1,j) = est.K_cam*[0;1/chi(3*i,j);0];
        elseif m==3
          ukf.dhdx(ind1,j) = est.K_cam*[-chi(3*i-2,j)/(chi(3*i,j)^2);...
            -chi(3*i-1,j)/(chi(3*i,j)^2);
            0];
        end
      end
      ukf.dhdx_sum = ukf.dhdx*ukf.weights(1,:)';
      ukf.dhdxDiff = repmat(ukf.dhdx_sum,1,ukf.n_a)-ukf.dhdx;
      % Partial Covariance Matrices
      dSig_xy = ukf.chiDiff*ukf.Wc*ukf.dhdxDiff';
      dSig_yy = ukf.dhdxDiff*ukf.Wc*ukf.yDiff'+...
        ukf.yDiff*ukf.Wc*ukf.dhdxDiff';
      % Compute Gradient for Each State
      gamma(3*i-2+(m-1)) = ...
        trace(-dSig_xy/Sig_yy*Sig_xy'-...
        Sig_xy/(-Sig_yy)*dSig_yy/Sig_yy*Sig_xy'-...
        Sig_xy/Sig_yy*dSig_xy');
    end
  end
  
  % Multiple Feature Normalized Sum
  gain=zeros(3,1);
  gSave=zeros(3,1);
  % testing correction term
  %    Correction = K*(est.y(:,cam.tInd-1)-ukf.yHat(:,cam.tInd-1));
  for i = 1:n
    ind2 = 3*i-2:3*i;
    uInd2 = 3*est.updateID(i)-2:3*est.updateID(i);
    g = gamma(ind2,1);
    gSave = gSave+g;
    gain = (est.expOmega-eye(3))*P(uInd2)+g+gain;
  end
  gain(3)=0;
  ctrl.uTemp = gain./norm(gain);
  est.traceVector(1:3,cam.tInd) = gSave;
  
  % Normalized Control Input
  ctrl.nu(:,cam.tInd) = ctrl.Gamma*ctrl.uTemp/(norm(ctrl.uTemp)+ctrl.epsilon);
  
  % Drive straight at first
  if cam.tInd==2
    ctrl.nu(:,cam.tInd) = ctrl.Gamma*ctrl.nu(:,cam.tInd)/(norm(ctrl.nu(:,cam.tInd)));
    %       ctrl.nu(:,cam.tInd) = 0.9*[0.1;0;0];
  elseif cam.tInd>2 && cam.tInd<=10
    %       ctrl.nu(:,cam.tInd) = ctrl.vStart;
    ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd-1);
  end
  
  p.ctrlTime(1,cam.tInd) = toc;
  
end

% % Drive straight at first
% if cam.tInd==1
% elseif cam.tInd<5
% %   ctrl.nu(:,cam.tInd) = ctrl.k_des*ctrl.vStart;
%    ctrl.nu(:,cam.tInd) = ctrl.nu(:,cam.tInd-1);
% end

