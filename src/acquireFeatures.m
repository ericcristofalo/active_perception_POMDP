%--------------------------------------------------------------------------
%
% File Name:      acquireFeatures.m
% Date Created:   2016/09/16
% Date Modified:  2016/09/16
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Acquire new virtual image and extract tracked feature
%                 points
%
%--------------------------------------------------------------------------

%% Acquire Virtual Image and Extract/Match Features

% Update Camera's Tranformation For Virtual Image Acquisition
cam.pose(1:3,1) = rob.pose(1:3,tInd)+trans.R_wr*(cam.offset(1:3,1));
trans.R_wc = trans.R_wr*trans.R_rc; % camera points to robot, then robot to world points
trans.R_cw = inv(trans.R_wc);
cam.pose(4:6,1) = rot2euler(trans.R_wc);
trans.camTrans = [trans.R_cw,-trans.R_cw*rob.pose(1:3,tInd); ...
   0,0,0,1]; % using inverse of camera to world (world to camera transformation)

% Acquire Virtual Image
obj.pixelPts = takeImage(obj.points, trans.camTrans, cam.K, cam.imSize);

% Save Pixel Points for Plotting
% obj.pixelPtsDiff = size(obj.points,2)-size(obj.pixelPts,2);
% if obj.pixelPtsDiff > 0 % fill size for p.pixelPts
%    p.pixelPts(:,:,tInd) = [obj.pixelPts,-100*ones(4,obj.pixelPtsDiff)];
% else
%    p.pixelPts(:,1:size(obj.pixelPts,2),tInd) = obj.pixelPts;
% end

% Kill If Any Features Are Lost
if size(obj.pixelPts,2)~=obj.n
%    warning('At least one image feature is now out of view.')
%    p.simulationError = 1;
end
% Kill If All Features Are Lost
if isempty(obj.pixelPts)
   warning('All image features are now out of view.')
   p.simulationError = 1;
end
% Kill is Error is Determined While Running Individual Simulation
if (compBool==0 && monteCarloBool==0) && p.simulationError==1
   return;
end

% Detect Current Tracked Features in image
obj.newFeatures = [];
for i = 1:obj.n
   obj.checkPts = obj.pixelPts(4,:)==p.pixelTrackPts(4,i,cam.tInd-1);
   if sum(obj.checkPts)~=0 % if points are found
      obj.pixelTrackPts(:,i) = obj.pixelPts(:,obj.checkPts);
   else % else choose new points
      obj.newFeatures = [obj.newFeatures,i];
   end
end
% Detect New Features in Image to Track
if ~isempty(obj.newFeatures)
   
   % Not Exploring
   if est.explore==0 || est.explore==1
      % Assign Blank Features When They Are Lost
      for i = 1:size(obj.newFeatures,2)
         obj.pixelTrackPts(1:3,obj.newFeatures(i)) = [0;0;1];
         obj.pixelTrackPts(4,obj.newFeatures(i)) = p.pixelTrackPts(4,obj.newFeatures(i),cam.tInd-1);
      end
      % Quit When Original Features Are Gone
      if size(obj.newFeatures,2)==obj.n
         if est.explore==0
%             disp('Camera has lost the original features');
            disp(['Time Index: ',num2str(tInd)]);
%             break; % kill the simulation
         elseif est.explore==1
            p.addFeatureSet = 1; % add new features
         end
      end
   end
   
   % OK to Add New Features
   if est.explore==2 || p.addFeatureSet==1
      disp('Selecting new feature(s) to track');
      % Randomly Acquire Points to Track
      obj.smallWindowPnts = obj.pixelPts(:,obj.pixelPts(1,:)<(cam.imSize(2)-cam.imSize(2)*obj.winFraction) & ...
         obj.pixelPts(1,:)>(cam.imSize(2)*obj.winFraction) & obj.pixelPts(2,:)<(cam.imSize(1)-cam.imSize(1)*obj.winFraction) & ...
         obj.pixelPts(2,:)>(cam.imSize(1)*obj.winFraction));
      if isempty(obj.smallWindowPnts)
         disp('Camera has nothing to see');
         disp(['Time Index: ',num2str(tInd)]);
         return;
      end
      obj.newPointInd = randsample(obj.smallWindowPnts(4,:),size(obj.newFeatures,2)); % randomly sample image
      % Assign New Features
      for i = 1:size(obj.newFeatures,2)
         obj.checkPts = obj.pixelPts(4,:)==obj.newPointInd(i);
         obj.pixelTrackPts(:,obj.newFeatures(i)) = obj.pixelPts(:,obj.checkPts);
      end
      % Plot
      if p.plotting==1
         p.plotInd=p.plotVal-2;
      end
   end
   
end
% Add Simulated Image Feature Location Noise
if est.cameraNoise==1
   est.m_p(:,cam.tInd) = normrnd(0,est.camCov,[2*obj.n,1]);
   %       est.m_p
   for i = 1:obj.n
      ind = 2*i-1:2*i;
      if sum(obj.pixelTrackPts(1:2,i))~=0
         obj.pixelTrackPts(1:2,i) = obj.pixelTrackPts(1:2,i)+est.m_p(ind,cam.tInd);
      end
   end
end
% Save Feature History
p.pixelTrackPts(:,:,cam.tInd) = obj.pixelTrackPts; % save current features

