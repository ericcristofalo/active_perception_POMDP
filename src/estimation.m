%--------------------------------------------------------------------------
%
% File Name:      estimation.m
% Date Created:   2016/09/02
% Date Modified:  2016/09/16
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Estimation Script
%
%--------------------------------------------------------------------------


%% Estimation Initialization on cam.tInd==1
if (tInd==2 && cam.tInd==1)
   
% Feature Handling
if est.n==0
   est.n = obj.nTotal;
elseif obj.samePoint==2
   est.n = obj.n;
else
   est.n = obj.n;
end
est.nTemp = obj.n;
est.initID = [];
est.predictID = 1:obj.n;
est.updateID = 1:obj.n;
est.ind = zeros(est.n,cam.tSpanLength);
est.ind(1:est.nTemp,1) = obj.pixelTrackPts(4,:)';

% Estimation Indicator
cam.estInd = 1;

% Estimation
est.nTemp = est.n;
est.K_cam = cam.K(1:2,:);
% Noise Covariance Matrices
est.Q = est.q*eye(3*est.n,3*est.n);
est.R = est.r*eye(2*est.n,2*est.n);
% Measurements
est.y = zeros(2*est.n,cam.tSpanLength);
for i = 1:est.n
   est.y(2*i-1:2*i,1) = obj.pixelTrackPts(1:2,i);
end
% Linearized Dynamics (already linear)
est.Omega = skewSymMat(rob.v(4:6,1));
est.expOmega = expm(-est.Omega.*cam.dt); % matrix exponential
% Record Ground Truth Comparison
est.GT = zeros(3*est.n,cam.tSpanLength);
est.nTemp = sum(est.ind(:,cam.tInd)~=0);
temp = trans.camTrans*...
   [obj.points(1:3,est.ind((est.ind(:,cam.tInd)~=0),cam.tInd));...
   ones(1,est.n)];
for i = 1:est.n
   ind = 3*i-2:3*i;
   est.GT(ind,cam.tInd) = temp(1:3,i);
end
% Initial Error For Estimation
if monteCarloBool==1
   est.Pz_0 = 49*ones(1,est.n)*rand+1; % random guess
elseif compBool==1
   est.Pz_0 = obj.Pz_0(simInd,:);
else % constant error
   est.error = 10;     % meters
   est.Pz_0 = repmat(est.error,1,est.n);
%    est.Pz_0 = (obj.camTrackPts(3,:)+repmat(est.error,1,est.n));
%    est.Pz_0 = (-rob.pose(3,1)+20)*ones(1,est.n)';
end

% Timing
p.filterTime = zeros(3,cam.tSpanLength);
p.ctrlTime = zeros(1,cam.tSpanLength);

%--------------------------------------------------------------------------
if any(strcmp(s.estCond,'EKF'))
      
   % Covariance Matrices
   est.Sigma_ekf = 1E2*eye(3*est.nTemp);
   est.Sigma_ekf_ = est.Sigma_ekf;
   est.sigma_ekf = zeros(3*est.n,cam.tSpanLength);
   est.sigma_ekf(:,1) = diag(est.Sigma_ekf);
   % State Estimation
   est.P_ekf = zeros(3*est.n,cam.tSpanLength);
   for i = 1:est.n
      est.P_ekf(3*i-2:3*i,1) = cam.K_inv*[est.y(2*i-1:2*i,1);1].*est.Pz_0(i);
   end
   
   % Plot Filter
%    plotEKF;
end

%--------------------------------------------------------------------------
if any(strcmp(s.estCond,'UKF'))
   
   % Covariance Matrices
   est.Sigma_ukf = 1E2*eye(3*est.nTemp);
   est.Sigma_ukf_ = est.Sigma_ukf;
   est.sigma_ukf = zeros(3*est.n,cam.tSpanLength);
   est.sigma_ukf(:,1) = diag(est.Sigma_ukf);
   % State Estimation
   est.P_ukf = zeros(3*est.n,cam.tSpanLength);
   for i = 1:est.nTemp
      est.P_ukf(3*i-2:3*i,cam.tInd) = cam.K_inv*[est.y(2*i-1:2*i,1);1].*est.Pz_0(i);
   end
   
   ukf.rGain = 1; % must be set to 1
   
   % UT Transform Constants
   ukf.n_a = 2*(3*est.n)+1;
   ukf.weights = zeros(2,ukf.n_a);
   ukf.L = 3*est.n;
   
   %%% UT Transform (Wan et al. (Wikipedia))
   ukf.alpha = 0.001;
   ukf.L = 3*est.n;
   ukf.k = 0;
   ukf.lambda = ukf.alpha^2*(ukf.L+ukf.k)-ukf.L;
%    ukf.lambda = -ukf.L+1;
   ukf.c = ukf.L+ukf.lambda;
   ukf.beta = 2; % 2 is optimal for Gaussians
   ukf.weights(1,:) = [ukf.lambda/ukf.c 0.5/ukf.c+zeros(1,2*ukf.L)];  %weights for means
   ukf.weights(1,1) = ukf.lambda/(ukf.L+ukf.lambda);
   ukf.weights(2,:) = ukf.weights(1,:);
   ukf.weights(2,1) = ukf.lambda/(ukf.L+ukf.lambda)+(1-ukf.alpha^2+ukf.beta);
   %%% UT Transform (Uhlmann et al.)
%    ukf.lambda = 3; % (L+kappa) and 3 is optimal for Gaussians
%    ukf.kappa = ukf.lambda-ukf.L;
%    ukf.weights(1,1) = ukf.kappa/(ukf.L+ukf.kappa);
%    ukf.weights(1,2:end) = 1/(2*(ukf.L+ukf.kappa))*ones(1,ukf.L*2);
%    ukf.weights(2,:) = ukf.weights(1,:);
   %%% Simplified Weights
%    ukf.weights(1,1) = (ukf.n_a-3*est.n)/ukf.n_a;
%    ukf.weights(1,2:end) = 1/(2*ukf.n_a);
%    ukf.weights(2,:) = ukf.weights(1,:);

   % Generate Sigma Vectors
   ukf.chi = zeros(3*est.n,ukf.n_a,cam.tSpanLength);
   ukf.Cov_a = est.Sigma_ukf; 
%    ukf.Cov_a = [est.Sigma_ukf,zeros(3*est.n,2*est.n);zeros(2*est.n,3*est.n),est.R];
   % UT Transform (Wan et al. 2000 (Wikipedia))
   ukf.CovSquare = chol((ukf.L+ukf.lambda)*ukf.Cov_a)';
   % UT Transform (Uhlmann et al.)
%    ukf.CovSquare = chol((ukf.L+ukf.kappa)*ukf.Cov_a)';
%    ukf.lambda = 3;
%    ukf.CovSquare = chol(ukf.lambda*ukf.Cov_a)';
   % Final Sigma Vectors
   ukf.chi(:,:,cam.tInd) = repmat(est.P_ukf(:,cam.tInd),1,ukf.n_a)+[zeros(3*est.n,1),ukf.CovSquare,-ukf.CovSquare];

   % Propagated Measurement
   ukf.ySamples = zeros(2*est.n,ukf.n_a);
   for i = 1:est.n
      ind1 = 2*i-1:2*i;
      ind2 = 3*i-2:3*i;
      for j = 1:(ukf.n_a)
         ukf.ySamples(ind1,j) = 1/(ukf.chi(3*i,j,cam.tInd))*est.K_cam*ukf.chi(ind2,j,cam.tInd);
%          ukf.ySamples(ind1,j) = 1/(ukf.chi(3*i,j,cam.tInd))*est.K_cam*ukf.chi(ind2,j,cam.tInd)+est.chi(4:5,i,cam.tInd);
      end
   end
   ukf.yHat = zeros(2*est.n,cam.tSpanLength);
   ukf.yHat(:,cam.tInd) = ukf.ySamples*ukf.weights(1,:)';

   % Covariance Matrices
   diffP = ukf.chi(:,:,cam.tInd)-repmat(est.P_ukf(:,cam.tInd),1,ukf.n_a);
   diffy = ukf.ySamples-repmat(ukf.yHat(:,cam.tInd),1,ukf.n_a);
   Cov_yy = diffy*diag(ukf.weights(2,:))*diffy'+ukf.rGain*est.R;
   Cov_Py = diffP*diag(ukf.weights(2,:))*diffy';
   
%    % Final Update (Uhlmann et al. 1997)
   K = Cov_Py/Cov_yy;
   est.P_ukf(:,cam.tInd) = est.P_ukf(:,cam.tInd) + K*(est.y(:,cam.tInd)-ukf.yHat(:,cam.tInd)); % state update
   est.Sigma_ukf = est.Sigma_ukf_-K*Cov_yy*K'; % covariance update
   
   % Weights for Active UKF
   ukf.W = diag(ukf.weights(1,:));
   ukf.Wc = diag(ukf.weights(2,:));
   
end

%--------------------------------------------------------------------------
if any(strcmp(s.estCond,'LTVKF'))
      
   % Covariance Matrices
   est.Sigma_ltvkf = 1E2*eye(3*est.nTemp);
   est.Sigma_ltvkf_ = est.Sigma_ltvkf;
   est.sigma_ltvkf = zeros(3*est.n,cam.tSpanLength);
   est.sigma_ltvkf(:,1) = diag(est.Sigma_ltvkf);
   % State Estimation
   est.P_ltvkf = zeros(3*est.n,cam.tSpanLength);
   for i = 1:est.n
      est.P_ltvkf(3*i-2:3*i,1) = cam.K_inv*[est.y(2*i-1:2*i,1);1].*est.Pz_0(i);
   end

end

end % if (tInd==2 && cam.tInd==1)


%% Estimation on cam.tInd~=1

if (cam.tInd~=1 && cam.estInd~=cam.tInd)
%--------------------------------------------------------------------------
% Feature Handling Framework
%--------------------------------------------------------------------------

%    est.ind(:,cam.tInd-1) = [2890 1808 3 4]';
%    p.pixelTrackPts(4,:,cam.tInd) = [2890 2302];
%    p.pixelTrackPts(4,:,cam.tInd) = [1808 1];
%    %          est.ind(:,:,cam.tInd-1) = [7 8 5 11]';
%    %          p.pixelTrackPts(4,:,cam.tInd) = [5 12 13 7];
%    %          obj.n = 3;
%    est.ind(:,cam.tInd) = [0 0 0]';

   % Match Estimated Features
   est.initID=[]; est.updateID=[]; est.predictID = [];
   if est.explore==0 || (est.explore==1&&p.addFeatureSet==0)
      
      est.curPts = [];
      est.ind(:,cam.tInd) = est.ind(:,cam.tInd-1);
      est.predictID = find(est.ind(:,cam.tInd)~=0)';
      for i = 1:est.n
         est.checkPts = est.ind(i,cam.tInd-1)==p.pixelTrackPts(4,:,cam.tInd); % feature is still being estimated
         if sum(est.checkPts)==1
            if sum(p.pixelTrackPts(1:2,est.checkPts,cam.tInd))~=0
               est.updateID = [est.updateID,i];
            end
         end
      end
      
   end
   
   if est.explore==2 || p.addFeatureSet==1
      
      p.addFeatureSet = 0; % reset new features command
      
      % Keep Previous Tracked Point Estimates
      est.curPts = p.pixelTrackPts(4,:,cam.tInd);

      for i = 1:est.n
         % Check Previous Estimated Point Indices
         est.checkPts = est.ind(i,cam.tInd-1)==p.pixelTrackPts(4,:,cam.tInd);
         if sum(est.checkPts)==1 % feature is still being estimated
            est.ind(i,cam.tInd) = p.pixelTrackPts(4,est.checkPts,cam.tInd);
            est.updateID = [est.updateID,i];
            est.curPts(:,est.checkPts) = -1;
         end
      end
      est.curPts(:,est.curPts==-1) = []; % remove currently tracked indices
      est.initID = zeros(1,size(est.curPts,2));
      % Replace Worst Estimates Points With Newly Acquired Points
      for i = 1:size(est.curPts,2)
         [est.newInd,est.newFeatures] = find(est.ind(:,cam.tInd)==0);
         % Find Worst Estimate
         est.worstEst = 0;
         for j = est.newInd'
            ind = 3*j-2:3*j;
            est.curEst = trace(est.Cov(ind,ind,cam.tInd-1));
            if est.curEst>est.worstEst
               est.worstInd = j;
               est.worstEst = est.curEst;
            elseif est.curEst==0
               est.worstInd = j;
               break;
            end
         end
         est.ind(est.worstInd,cam.tInd) = est.curPts(1,i);
         est.initID(i) = est.worstInd;
      end
      % Keep all Remaining Estimates from Previous Time Step
      est.ind(est.ind(:,cam.tInd)==0,cam.tInd) = est.ind(est.ind(:,cam.tInd)==0,cam.tInd-1);
      est.nTemp = sum(est.ind(:,cam.tInd)~=0);
      
      % Find Update Indices (current tracked features that are not initialized)
%       [tempInd,temp] = find(est.ind(:,cam.tInd)==0);
%       est.updateID = [est.updateID,tempInd'];
%       est.updateID(est.ind(:,cam.tInd)==0)=[];
      % Find Prediction Indices (previous/current features that are not initialized)
      if ~isempty(est.initID)
         for i = 1:est.n
            if est.ind(i,cam.tInd)~=0 && all(est.initID~=i)
               est.predictID = [est.predictID,i];
            end
         end
      else
         temp = 1:est.n;
         est.predictID = temp(est.ind(:,cam.tInd)~=0);
      end
      
   end
   
%    clear v y Pprev Pprev_ Q R Hcur hCur Pcur ind1 ind2;
   
   % Save Time-Varying Update-Sized Measurements
%    est.nP = size(est.predictID,2);
   updateMat = zeros(3*est.nTemp,3*est.nTemp);
   for i = est.updateID
      ind = find(est.predictID==i);
      uInd1 = 2*ind-1:2*ind;
      uInd2 = 3*ind-2:3*ind;
      curInd = est.ind(i,cam.tInd);
      
      est.y(uInd1,cam.tInd) = ...
         p.pixelTrackPts(1:2,p.pixelTrackPts(4,:,cam.tInd)==curInd,cam.tInd);
      updateMat(uInd2,uInd2) = eye(3);
   end
   
   % Extract Current Prediction Subsystem
   pInd1=[]; pInd2=[];
   for i = est.predictID
      pInd1 = [pInd1,2*i-1:2*i];
      pInd2 = [pInd2,3*i-2:3*i];
   end
   
   % Extract Current Update Subsystem
   uInd1=[]; uInd2=[];
   for i = est.updateID
      uInd1 = [uInd1,2*i-1:2*i];
      uInd2 = [uInd2,3*i-2:3*i];
   end
   
   % Current Update Size
   n = size(est.updateID,2);
   
   
%--------------------------------------------------------------------------
% Normal Linear Prediction Setup For Each Filter
%--------------------------------------------------------------------------

   % Extract Dynamics
   est.Omega = skewSymMat(rob.v(4:6,tInd)); % original
%    est.Omega = skewSymMat(sum(rob.v(4:6,tInd-cam.val:tInd),2)/(cam.val+1)); % modified to use finer incriments
   est.expOmega = expm(-est.Omega*cam.dt); % matrix exponential
   
%    est.expOmega = euler2rot(cam.pose(4:6,cam.tInd))'*...
%                   euler2rot(cam.pose(4:6,cam.tInd-1)); % relative orientation method
%    est.expOmega
%    disp('--------')
   expOmegaFull = kron(eye(est.nTemp),est.expOmega);
   v = repmat(rob.v(1:3,tInd),est.nTemp,1); % original
%    v = repmat(sum(rob.v(1:3,tInd-cam.val:tInd),2)/(cam.val+1),est.nTemp,1); % modified to use finer incriments
   y = est.y(uInd1,cam.tInd);
%    CovPrev = est.Cov(pInd2,pInd2,cam.tInd-1);
   Q = est.Q(pInd2,pInd2);
   R = est.R(uInd1,uInd1);


%--------------------------------------------------------------------------
% Filter Estimation
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
   if any(strcmp(s.estCond,'EKF'))
      
      % Start EKF Timer
      tic;
      
      % Discrete EKF Prediction
      Pprev = est.P_ekf(pInd2,cam.tInd-1);
      Pcur_ = expOmegaFull*Pprev-v.*cam.dt;
      est.Sigma_ekf_(pInd2,pInd2) = ...
         expOmegaFull*est.Sigma_ekf*expOmegaFull'+Q;

      % Discrete EKF Update
      Hcur = zeros(2*n,3*n);
      hCur = zeros(2*n,1);
      ekf.G = zeros(2*n,2*n);
      ekf.K = zeros(3*n,2*n);
      for i = 1:n
         ind1a = 2*i-1:2*i;
         ind2a = 3*i-2:3*i;
         P = Pcur_(uInd2(ind2a),1);
         Hcur(ind1a,ind2a) = est.K_cam*...
            [1/P(3), 0, -P(1)/P(3)^2;
            0, 1/P(3), -P(2)/P(3)^2;
            0, 0, 0];
         hCur(ind1a,1) = 1/P(3)*est.K_cam*P;
      end
      % Kalman Gain Update
      ekf.G = (Hcur*est.Sigma_ekf_(uInd2,uInd2)*Hcur'+R);
      ekf.K = est.Sigma_ekf_(uInd2,uInd2)*Hcur'/ekf.G;
      % State Estimate Update
      ekf.correction = ekf.K*(y-hCur);
      Pcur = Pcur_(uInd2,1)+ekf.K*(y-hCur);
      % Prediction Covariance Update
      est.Sigma_ekf = est.Sigma_ekf_; % always save predictions
      est.Sigma_ekf(uInd2,uInd2) = est.Sigma_ekf_(uInd2,uInd2)-...
         ekf.K*Hcur*est.Sigma_ekf_(uInd2,uInd2);
      
      % Save Estimation
      est.P_ekf_(pInd2,cam.tInd) = Pcur_;
      est.P_ekf(pInd2,cam.tInd) = Pcur_; % always save predictions
      est.P_ekf(uInd2,cam.tInd) = Pcur;
      est.sigma_ekf(:,cam.tInd) = diag(est.Sigma_ekf);
      
      % End EKF Timer
      p.filterTime(1,cam.tInd) = toc;
      
      % Initialize New Features
      %    est.newInd = [];
      if ~isempty(est.curPts) % is new points are found
         % disp('Initializing new points');
         %       [est.newInd,est.newFeatures] = find((est.ind(:,cam.tInd)-est.ind(:,cam.tInd-1))~=0);
         ekf.PzGuess = mean(est.P_ekf(3*(1:est.nTemp-1),cam.tInd-1));
         for i = est.initID
            est.newFeature = p.pixelTrackPts(:,p.pixelTrackPts(4,:,cam.tInd)==est.ind(i,cam.tInd),cam.tInd);
            ind = 3*i-2:3*i;
            est.P_ekf_(ind,cam.tInd) = inv(cam.K)*[est.newFeature(1:2,1);1].*ekf.PzGuess;
            est.P_ekf(ind,cam.tInd) = est.P_ekf_(ind,cam.tInd);
            est.Pz_0(i) = est.P_(3*i,cam.tInd);
         end
      end
      
   end
   

%--------------------------------------------------------------------------
   if any(strcmp(s.estCond,'UKF'))

      % Start UKF Timer
      tic;

      % Discrete EKF Prediction
      Pprev = []; Pcur_ = []; Pcur = [];
      Pprev = est.P_ukf(pInd2,cam.tInd-1);
      Pcur_ = expOmegaFull*Pprev-v.*cam.dt;
      
      % Prediction Covariance Prediction
      est.Sigma_ukf_(pInd2,pInd2) = ...
         expOmegaFull*est.Sigma_ukf*expOmegaFull'+Q;
      
      % Temp for UKF only
      n = est.n;
      uInd1 = pInd1;
      uInd2 = pInd2;
      R = est.R(uInd1,uInd1);
      y = est.y(uInd1,cam.tInd);
      n_a = 2*3*n+1;
      
      % Generate Sigma Vector
      ukf.Cov_a = est.Sigma_ukf_(pInd2,pInd2);
%       ukf.Cov_a = [ukf.Cov_a];%,zeros(3*est.n,2*est.n);zeros(2*est.n,3*est.n),R];      
      % UT Transform (Wan et al. 2000 (Wikipedia))
      try
         ukf.CovSquare = chol((ukf.L+ukf.lambda)*ukf.Cov_a)';
      catch
         p.simulationError=1;
         warning('UKF Covariance not positive definite. Skipping simulation. ');
         return;
      end
      % UT Transform (Uhlmann et al. 1997)
%       ukf.CovSquare = chol((ukf.L+ukf.kappa)*ukf.Cov_a)';
%       ukf.CovSquare = chol(ukf.lambda*ukf.Cov_a)';
      
      ukf.chi(:,:,cam.tInd) = repmat(Pcur_,1,ukf.n_a)+[zeros(3*est.n,1),ukf.CovSquare,-ukf.CovSquare];
      
      % Propagated Measurement
      ukf.ySamples = zeros(2*est.n,ukf.n_a);
      for i = 1:est.n
         ind1 = 2*i-1:2*i;
         ind2 = 3*i-2:3*i;
         for j = 1:(ukf.n_a)
            ukf.ySamples(ind1,j) = 1/(ukf.chi(3*i,j,cam.tInd))*est.K_cam*ukf.chi(ind2,j,cam.tInd);%+est.chi(4:5,i,tInd);
         end
      end
      ukf.yHat(:,cam.tInd) = ukf.ySamples*ukf.weights(1,:)';
      
      % Covariance Matrices
      diffP = ukf.chi(:,:,cam.tInd)-repmat(Pcur_,1,ukf.n_a);
      diffy = ukf.ySamples-repmat(ukf.yHat(:,cam.tInd),1,ukf.n_a);
      Cov_yy = diffy*diag(ukf.weights(2,:))*diffy'+ukf.rGain*R;
      Cov_Py = diffP*diag(ukf.weights(2,:))*diffy';
      
      % Final Update (Uhlmann et al. 1997)
      K = Cov_Py/Cov_yy;
      Pcur = Pcur_ + updateMat*K*(y-ukf.yHat(:,cam.tInd));      % state update
      est.Sigma_ukf = est.Sigma_ukf_-updateMat*K*Cov_yy*K'; % covariance update
      
      % Save Data
%       est.P_ukf_(pInd2,cam.tInd) = Pcur_;
      est.P_ukf(pInd2,cam.tInd) = Pcur;
      est.sigma_ukf(pInd2,cam.tInd) = diag(est.Sigma_ukf);
      
      % End UKF Timer
      post.filterTime(3,cam.tInd) = toc;
      
      % Output:
%       est.P_ukf(:,tInd)
%       Cov
      
   end
   
%--------------------------------------------------------------------------
   if any(strcmp(s.estCond,'LTVKF'))
      
      % Start LTVKF Timer
      tic;
      
      % Discrete LTVKF Prediction
      Pprev = est.P_ltvkf(pInd2,cam.tInd-1);
      Pcur_ = expOmegaFull*Pprev-v.*cam.dt;
      est.Sigma_ltvkf_(pInd2,pInd2) = ...
         abs(expOmegaFull*est.Sigma_ltvkf*expOmegaFull'+Q);

      % Discrete LTVKF Update
      Hcur = zeros(2*n,3*n);
      ltvkf.G = zeros(2*n,2*n);
      ltvkf.K = zeros(3*n,2*n);
      for i = 1:n
         ind1a = 2*i-1:2*i;
         ind2a = 3*i-2:3*i;
%          P = Pcur_(uInd2(ind2a),1);
         ycur = y(uInd1(ind1a),1);
%          Hcur(ind1a,ind2a) = ...
%             est.K_cam - [0,0,ycur(1);0,0,ycur(2)];
         Hcur(ind1a,ind2a) = ...
            [est.K_cam(1,1),0,est.K_cam(1,3)-ycur(1);
             0,est.K_cam(2,2),est.K_cam(2,3)-ycur(2)];
      end
      hCur = Hcur*Pcur_;
      % Kalman Gain Update
      ltvkf.G = inv(Hcur*est.Sigma_ltvkf_(uInd2,uInd2)*Hcur'+R);
      ltvkf.K = est.Sigma_ltvkf_(uInd2,uInd2)*Hcur'*ltvkf.G;
      % State Estimate Update with Virtual Measurement
      ltvkf.correction = ltvkf.K*(-hCur);
      Pcur = Pcur_(uInd2,1)+ltvkf.K*(-hCur);
      % Prediction Covariance Update
      est.Sigma_ltvkf = est.Sigma_ltvkf_; % always save predictions
      est.Sigma_ltvkf(uInd2,uInd2) = abs(est.Sigma_ltvkf_(uInd2,uInd2)-...
         ltvkf.K*Hcur*est.Sigma_ltvkf_(uInd2,uInd2));
      
      % Save Estimation
      est.P_ltvkf_(pInd2,cam.tInd) = Pcur_;
      est.P_ltvkf(pInd2,cam.tInd) = Pcur_; % always save predictions
      est.P_ltvkf(uInd2,cam.tInd) = Pcur;
      est.sigma_ltvkf(:,cam.tInd) = diag(est.Sigma_ltvkf);
      
      % End EKF Timer
      p.filterTime(1,cam.tInd) = toc;
      
      % Initialize New Features
      %    est.newInd = [];
      if ~isempty(est.curPts) % is new points are found
         % disp('Initializing new points');
         %       [est.newInd,est.newFeatures] = find((est.ind(:,cam.tInd)-est.ind(:,cam.tInd-1))~=0);
         ltvkf.PzGuess = mean(est.P_ltvkf(3*(1:est.nTemp-1),cam.tInd-1));
         for i = est.initID
            est.newFeature = p.pixelTrackPts(:,p.pixelTrackPts(4,:,cam.tInd)==est.ind(i,cam.tInd),cam.tInd);
            ind = 3*i-2:3*i;
            est.P_ltvkf_(ind,cam.tInd) = inv(cam.K)*[est.newFeature(1:2,1);1].*ltvkf.PzGuess;
            est.P_ltvkf(ind,cam.tInd) = est.P_ltvkf_(ind,cam.tInd);
            est.Pz_0(i) = est.P_(3*i,cam.tInd);
         end
      end
      
   end

   cam.estInd = cam.tInd;
end % if (cam.tInd~=1 && cam.estInd~=cam.tInd)


%% Record Ground Truth Comparison
temp = trans.camTrans*...
   [obj.points(1:3,obj.pixelTrackPts(4,:)');...
   ones(1,est.n)];
for i = 1:est.n
   ind = 3*i-2:3*i;
   est.GT(ind,cam.tInd) = temp(1:3,i);
end

   