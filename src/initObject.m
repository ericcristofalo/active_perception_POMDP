%--------------------------------------------------------------------------
%
% File Name:      initObject.m
% Date Created:   2016/09/02
% Date Modified:  2017/10/15
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Virtual Object Scene Initialization
%
%--------------------------------------------------------------------------

if ( compBool==0 )
   % Read 3D Model
   if strcmp(stl.filename,'random')
      obj.randV = [1.5,1.5,-3];
      obj.v = [obj.randV(1)*2,0,0;0,obj.randV(2)*2,0;0,0,obj.randV(3)]*rand(3,obj.sample-1)-...
         repmat([obj.randV(1);obj.randV(2);0],1,obj.sample-1);
      obj.v = [[[0;0;0],obj.v];linspace(1,obj.sample,obj.sample)];
      
   else % read .stl file
      
      % Read, Orient, and Assign Indices to Point Cloud Model
      [stl, obj] = stlLibrary(stl, obj);
      stl.v = []; stl.f = []; stl.n = []; stl.c = [];
      
   end
end

% % % % Generate Point Ground Plane on XY Plane
% % % obj.plane.max = ceil(max(abs(max(obj.v(1,:))-min(obj.v(1,:))),...
% % %                          abs(max(obj.v(2,:))-min(obj.v(2,:))))*0.6);
% % % obj.plane.corners = [-obj.plane.max,obj.plane.max,-obj.plane.max,obj.plane.max];
% % % obj.plane.dist = obj.plane.max/20;
% % % obj.plane.points = generatePlane(obj.plane.corners, obj.plqane.dist);
% % % % Combine Points
% % % obj.plane.points(4,:) = obj.plane.points(4,:)+max(obj.v(4,:));
% % % obj.points = [obj.v,obj.plane.points];

% No Ground Plane Option
obj.points = obj.v;
obj.nTotal = size(obj.points,2);

% Add Origin Point as First Point
if obj.samePoint==2
   obj.points = [obj.points,[0;0;0;obj.points(4,end)+1]];
end

% Acquire Virtual Image
obj.pixelPts = takeImage(obj.points, trans.camTrans,...
   cam.K, cam.imSize);
% p.pixelPts = zeros(4,length(obj.points),s.tSpanLength);  % save pixelPts

% Check If Minimum Number of Points are Visible (For Monte Carlo
% Simulations)
while size(obj.pixelPts,2)<obj.n
   % Add World Points
   obj.newPoint = [obj.randV(1)*2,0,0;0,obj.randV(2)*2,0;0,0,obj.randV(3)]*rand(3,1)-...
         [obj.randV(1);obj.randV(2);0];
   obj.points = [obj.points,[obj.newPoint;obj.points(4,end)+1]];
   % Acquire Virtual Image
   obj.pixelPts = takeImage(obj.points, trans.camTrans,...
   cam.K, cam.imSize);
end

% Select Points to Track
if obj.samePoint==1
   if obj.n==1
      obj.newPointInd = 1;
   elseif obj.n==2
      obj.newPointInd = [1,2];
   elseif obj.n==8
      obj.newPointInd = [1,2,3,4,5,6,7,8];
   elseif obj.n==10
      % initialIndices = obj.pixelTrackPts(4,:);
      % save('Initial_Conditions/trackPtsIndices_10_colosseum.mat','initialIndices')
      load('Initial_Conditions/trackPtsIndices_10_colosseum.mat');
      obj.newPointInd = initialIndices;
   end
else
   % Randomly Acquire Points to Track
%    obj.winFraction = 0.4;
%    obj.winFraction = 0.2;
   obj.winFraction = 0.0;
   obj.smallWindowPnts = obj.pixelPts(:,obj.pixelPts(1,:)<(cam.imSize(2)-cam.imSize(2)*obj.winFraction) & ...
      obj.pixelPts(1,:)>(cam.imSize(2)*obj.winFraction) & obj.pixelPts(2,:)<(cam.imSize(1)-cam.imSize(1)*obj.winFraction) & ...
      obj.pixelPts(2,:)>(cam.imSize(1)*obj.winFraction));
   obj.newPointInd = randsample(obj.smallWindowPnts(4,:),obj.n); % randomly sample image
end
% Acquire Desired Points to Track in Camera View
obj.pixelTrackPts = zeros(4,obj.n);
for i = 1:obj.n
   obj.checkPts = obj.pixelPts(4,:)==obj.newPointInd(i);
   obj.pixelTrackPts(:,i) = obj.pixelPts(:,obj.checkPts);
end
% Add Origin Point as First Point
if obj.samePoint==2
   obj.checkPts = obj.pixelPts(4,:)==obj.points(4,end);
   obj.pixelTrackPts(:,1) = obj.pixelPts(:,obj.checkPts);
end
% Sort pixelTrackPoints on First Iteration
[val,ind] = sort(obj.pixelTrackPts(4,:));
obj.pixelTrackPts = obj.pixelTrackPts(:,ind);

% % Add Simulated Image Feature Location Noise
% if est.cameraNoise==1
% %    est.m_p = normrnd(0,est.r,[2,obj.n]).*1;
%    est.m_p = normrnd(0,est.camCov,[2,obj.n]);
% % 	est.m_p;
%    for i = 1:obj.n
%       if sum(obj.pixelTrackPts(1:2,i))~=0
%          obj.pixelTrackPts(1:2,i) = obj.pixelTrackPts(1:2,i)+est.m_p(:,i);
%       end
%    end
% end
p.pixelTrackPts = zeros(4,obj.n,cam.tSpanLength); % save pixelTrackPts
p.pixelTrackPts(:,:,cam.tInd) = obj.pixelTrackPts;

% Save Pixel Points for Plotting
% obj.pixelPtsDiff = size(obj.points,2)-size(obj.pixelPts,2);
% if obj.pixelPtsDiff > 0 % fill size for p.pixelPts
%    p.pixelPts(:,:,cam.tInd) = [obj.pixelPts,-100*ones(4,obj.pixelPtsDiff)];
% else
%    p.pixelPts(:,1:size(obj.pixelPts,2),cam.tInd) = obj.pixelPts;
% end

% Calculate True Depth
obj.worldTrackPts = obj.points(:,p.pixelTrackPts(4,:,cam.tInd)); % corresponding world points
obj.camTrackPts = ...
   trans.camTrans*[obj.worldTrackPts(1:3,:);ones(1,obj.n)];
p.depthInit = obj.camTrackPts(3,:);

