%--------------------------------------------------------------------------
%
% File Name:      postMonteCarloTrials.m
% Date Created:   2016/02/20
% Date Modified:  2016/09/25
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Post Process Monte Carlo Trials
%
%--------------------------------------------------------------------------

%% Extract Monte Carlo Data

% Check For Existing Folder in Matlab Variables
if ~exist('pxSave','var')
   folderPath = [pwd,'/Monte_Carlo_Trials/','20171105_204134','/'];
   if exist([folderPath,'pixelData.mat'],'file')~=0
      
      % Determine Number of Simulations
      sims = dir(folderPath);
      numSims = 0;
      for i = 1:size(sims,1)
         if strcmp(sims(i).name(1),'s')
            numSims = numSims+1;
         end
      end
      
      % Manually Extract Pixel Data if Not Saved Already
      for simInd = 1:numSims
         
         disp(['Extracting Pixel Data: ',num2str(simInd)]);
         
         % Load .mat
         clear rob cam est ctrl post trans tInd;
         load([folderPath,'/sim_',num2str(simInd),'.mat']);
         
         % Initialize Matrices
         if simInd==1
            pxSave = zeros(numSims, cam.tSpanLength);
            pySave = zeros(numSims, cam.tSpanLength);
            trajX = zeros(numSims, cam.tSpanLength);
            trajY = zeros(numSims, cam.tSpanLength);
            trajZ = zeros(numSims, cam.tSpanLength);
         end

         % Save Features
         px = reshape(p.pixelTrackPts(1,1,1:cam.tInd),1,cam.tInd);
         py = reshape(p.pixelTrackPts(2,1,1:cam.tInd),1,cam.tInd);
         diff = cam.tSpanLength-cam.tInd;
         pxSave(simInd,:) = [px,-1*ones(diff,1)'];
         pySave(simInd,:) = [py,-1*ones(diff,1)'];
         
         % Save Camera Trajectories
         diff = cam.tSpanLength-round(tInd*0.005);
         ind = (1:round(tInd*0.005))./0.005;
         trajX(simInd,:) = [rob.pose(1,ind),zeros(diff,1)'];
         trajY(simInd,:) = [rob.pose(2,ind),zeros(diff,1)'];
         trajZ(simInd,:) = [rob.pose(3,ind),zeros(diff,1)'];
      end
      
      % Save Monte Carlo Run
      filename = [folderPath,'/pixelData.mat'];
      save(filename,'pxSave','pySave','trajX','trajY','trajZ');
      
   else
      
      % Load .mat
      load([folderPath,'/pixelData.mat']);
      
   end
end


% %% Find Mean Final Circle Radius
% 
% rMean = 0;
% rMean2 = 0;
% num2 = 0;
% numRegions = 10;
% Count = zeros(numRegions+1,numRegions+1);
% rTotal1 = [];
% rTotal2 = [];
% for simInd = 1:numSims
%    
%    simInd
%    
%    % Extract data
%    px = pxSave(simInd,:);
%    px(px==-1)=[];
%    py = pySave(simInd,:);
%    py(py==-1)=[];
%    
%    % Shift Origin to Center
%    px = px-640/2;
%    py = py-480/2;
%    
%    % Caluclate mean radius of each sample trajectory
% %    nSamples = 1000;
% %    if px>nSamples
% %       px = px(end-nSamples:end);
% %       py = py(end-nSamples:end);
% %    end
%    rCur = mean((px.^2+py.^2).^(0.5));
%    rMean = rCur + rMean;
%    
%    % Calculate the radius of each point
%    rMean2 = sum((px.^2+py.^2).^(0.5)) + rMean2;
%    num2 = length(px)+num2;
%    
%    % Save each radius
%    rCur = (px.^2+py.^2).^(0.5);
% %    thetaCur = atan(py./px);
% %    rCur = rCur(thetaCur<0.2&thetaCur>(-0.2));
%    rCur = rCur(py>-240&py<240);
%    if ~isempty(rCur)
% %       rTotal1 = [rTotal1,rCur];
%       
%       rCurTemp = mean(rCur);
%       rTotal1 = [rTotal1,rCur];
%    end
%    
% %    % Calculate Probabilities
% %    startRad = (px(1)^2+py(1)^2)^(0.5);
% %    endRad = (px(end)^2+py(end)^2)^(0.5);
% %    for regInd = 1:numRegions+1
% %       prevRadius = (regInd-1)*640/2/numRegions;
% %       if regInd<=numRegions
% %          curRadius = 640/2/numRegions+(regInd-1)*640/2/numRegions;
% %       else
% %          curRadius = inf;
% %       end
% %       
% %       if startRad>prevRadius && startRad<curRadius
% %          ind1 = regInd;
% %       end
% %       
% %       if endRad>prevRadius && endRad<curRadius
% %          ind2 = regInd;
% %       elseif px(end)==320 || py(end)==240
% %          ind2 = numRegions+1;
% %       end
% %    end
% %    Count(ind1,ind2) = Count(ind1,ind2)+1;
% 
%    
% end
% % Mean radius of all trajectories
% rMean = rMean/numSims
% % Mean radius of all points
% rMean2 = rMean2/num2
% 
% % % Probability
% % Count
% % Prob = zeros(numRegions+1,numRegions+1);
% % for i = 1:numRegions+1
% %    for j = 1:numRegions+1
% %       Prob(i,j) = Count(i,j)/sum(Count(i,:));
% %    end
% % end
% % Prob
% % 
% % [X,Y] = meshgrid(1:1:numRegions+1);
% % figure(1), mesh(X,Y,Prob);
% 
% % Distribution of Radii
% figure(2); clf(2); hist(rTotal1,1000);
% xlabel('Circle Radius (pixels)'); ylabel('Number of Features');
% pos = get(2,'Position');
% set(2,'Position',[pos(1:2),640,480]);
% 
% % Fix Axes Font and Size
% figureHandleRange = get(0,'Children')';
% set(findall(figureHandleRange,'-property','FontSize'),'FontSize',20);
% set(findall(figureHandleRange,'-property','FontName'),'FontName','Times')
% axisFontSize = 14;
% 
% % Global Pose
% figure(2);
% pos = get(2,'Position');
% set(2,'Position',[pos(1:2),560,420]);
% set(gca,'FontSize',axisFontSize)


%% Virtual Images

figure(10); clf(10); hold on; box on;
axis equal;
axis([0, 1000, 0, 1000]);

% pos = get(10,'Position');
% set(10,'Position',[pos(1:2),1120,801]);

displayInd = 0;
displayNum = 2;
for simInd = 1:numSims
   
   displayInd = displayInd+1;
   if displayInd~=displayNum
      continue;
   else
      displayInd = 0;
      disp(['Plotting Image Features: ',num2str(simInd)]);
   end
   
% %    % plotImage(post.pixelPts(:,:,tInd), post.pixelTrackPts(:,:,tInd), cam.plotSize, obj.color.pts, obj.color.trackPts);
%    plotImage(obj.pixelPts, post.pixelTrackPts(:,:,1), cam.plotSize, obj.color.pts, [0,1,0]);
%    plotImage(obj.pixelPts, post.pixelTrackPts(:,:,tInd), cam.plotSize, obj.color.pts, [1,0,0]);
% %    % title('Virtual Camera Image');

   % Plot Virtual Image
%    plot(pixelPoints(1,:),pixelPoints(2,:),'Color',pointColor,...
%       'LineStyle','none','Marker','.');

   % Plot the Feature Trajectory in Virtual Image
   px = pxSave(simInd,:);
   py = pySave(simInd,:);
   
   % Remove Non-saved Values
   px(px==-1)=[];
   py(py==-1)=[];
   
   px(px==0)=[];
   py(py==0)=[];
   
   % Remove Ugly Lines
%    max(px)
%    max(py)
%    min(px)
%    min(py)
%    a = norm([max(px)-min(px),max(py)-min(py)]);
%    b = min(abs(diff(px)));
%    c = min(abs(diff(py)));
%    if b<0.1 && c<0.1
%       continue
%    end

   % Start and finish locations
   scatter(px(1,1),py(1,1),30,[0,1,0],'fill');
   scatter(px(1,end),py(1,end),30,[1,0,0],'fill');
   
%    plot(px,py,'Color',[0.8,0.8,1],'LineWidth',1);
   
   z = zeros(1,size(px,2));
   S = surface([px;px],[py;py],[z;z],...
            'facecol','no',...
            'edgecol','interp',...
            'linew',2,...
            'edgealpha',0.1,...
            'edgecolor','b');
   
   
end

% xlabel('x-axis (pixels)'); ylabel('y-axis (pixels)'); 

set(gca,'YDir','reverse');
set(gca,'xtick',[])
set(gca,'xticklabel',[])
set(gca,'ytick',[])
set(gca,'yticklabel',[])
hold off;


%% Plot Global Trajectories

% Initialize Plotting
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
p.plotL = 2.0;
% Plotting Volume
p.plotVol = [-p.plotL,p.plotL,...
             -p.plotL,p.plotL,...
             -15,0];

figure(11); clf(11); 
hold on;
rotate3d on;
box on; grid on;
whitebg(obj.color.whitebg); % background color
          
% Generate Model For Plotting
obj.randV = [1.5,1.5,-3];
obj.sample = 10;
obj.v = [obj.randV(1)*2,0,0;0,obj.randV(2)*2,0;0,0,obj.randV(3)]*rand(3,obj.sample-1)-...
         repmat([obj.randV(1);obj.randV(2);0],1,obj.sample-1);
obj.v = [[[0;0;0],obj.v];linspace(1,obj.sample,obj.sample)];
obj.points = obj.v;

% Plot Model
plot3(obj.points(1,:),obj.points(2,:),obj.points(3,:),...
      '.','Color',obj.color.pts);
% obj.worldTrackPts = obj.points(:,p.pixelTrackPts(4,:,cam.tInd)); % corresponding world points
% scatter3(obj.worldTrackPts(1,:),...
%          obj.worldTrackPts(2,:),...
%          obj.worldTrackPts(3,:),30,obj.color.trackPts,'Fill');

% % Plot Robot
% plotQuad(rob.pose(:,tInd),0.5,0.3,[0,0,1]);

% Plot Trajectory
displayInd=0;
for i = 1:size(trajX,1);
   
   displayInd = displayInd+1;
   if displayInd~=displayNum
      continue;
   else
      displayInd = 0;
      disp(['Plotting Image Features: ',num2str(simInd)]);
   end
   
   disp(['Plotting Trajectories: ',num2str(i)]);
   
   x = trajX(i,trajX(i,:)~=(0));
   y = trajY(i,trajY(i,:)~=(0));
   z = trajZ(i,trajZ(i,:)~=(0));
   
%    % Plot Lines
%    plot3(x,y,z,'Color',[1,0.5,0.25],'LineWidth',1);
      
   % Plot Translucent Surface
   S = surface([x;x],[y;y],[z;z],...
            'facecol','no',...
            'edgecol','interp',...
            'linew',2,...
            'edgealpha',0.1,...
            'edgecolor','b');
end

% % Update Camera's Tranformation For Virtual Image Acquisition
% cam.pose(1:3,1) = rob.pose(1:3,tInd)+trans.R_wr*(cam.offset(1:3,1));
% trans.R_wc = trans.R_wr*trans.R_rc; % camera points to robot, then robot to world points
% trans.R_cw = inv(trans.R_wc);
% cam.pose(4:6,1) = rot2euler(trans.R_wc);
% trans.camTrans = [trans.R_cw,-trans.R_cw*rob.pose(1:3,tInd); ...
%    0,0,0,1]; % using inverse of camera to world (world to camera transformation)
% 
% % Plot Camera
% plotCoordSys(cam.pose(:,1), 50, [0,0,1], p.arrowLength, 2)
% plotCamera(cam.pose(:,1), cam.plotSize, [0,0,1], [0,0,0], p.arrowLength, 2)

% Plot Axes Labels
% title('Global Coordinate System');
xlabel('x-axis (m)'); ylabel('y-axis (m)'); zlabel('z-axis (m)');
axis equal;
view([1,0.8,0.8]); % good angle
% view([1,1,1]);
% view([-0.3,1,0.4]);
% view([0,0,1]); % top-down view
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');

% Compute New Axes Volume
p.plotVolCur = zeros(1,6);
p.plotVolCur(1,1) = -max([abs(obj.points(1,:)),abs(max(trajX))])-p.plotVol(1,2);
p.plotVolCur(1,2) = -p.plotVolCur(1,1);
p.plotVolCur(1,3) = -max([abs(obj.points(2,:)),abs(max(trajY))])-p.plotVol(1,4);
p.plotVolCur(1,4) = -p.plotVolCur(1,3);
p.plotVolCur(1,5) = min([abs(obj.points(3,:)),abs(min(trajZ))])-p.plotVol(1,5);
p.plotVolCur(1,6) = 0;
ind1 = find(p.plotVolCur([2,4,6])>p.plotVol([2,4,6]));
ind2 = find(p.plotVolCur([1,3,5])<p.plotVol([1,3,5]));
p.plotVolTemp = p.plotVol;
for i = ind1
   p.plotVolTemp(i*2) = p.plotVolCur(i*2);
end
for i = ind2
   p.plotVolTemp(i+1*(i-1)) = p.plotVolCur(i+1*(i-1));
end
axis(p.plotVolTemp);

% Plot Ground Patch
p.groundArea = [p.plotVolTemp(1,1) p.plotVolTemp(1,3);
               p.plotVolTemp(1,1) p.plotVolTemp(1,4);
               p.plotVolTemp(1,2) p.plotVolTemp(1,4);
               p.plotVolTemp(1,2) p.plotVolTemp(1,3)];
patch(p.groundArea(:,1),p.groundArea(:,2),obj.color.groundPlane,'EdgeColor','none');


%% Saving Figures

return;
save2pdf([folderPath,'MC_EKF_Pose_100'],11,1000);

% Fix Axes Font and Size
figureHandleRange = get(0,'Children')';
set(findall(figureHandleRange,'-property','FontSize'),'FontSize',20);
set(findall(figureHandleRange,'-property','FontName'),'FontName','Times')
axisFontSize = 14;
% Global Pose
figure(11);
pos = get(11,'Position');
set(11,'Position',[pos(1:2),560,420]);
set(gca,'FontSize',axisFontSize)
% Save PDF
save2pdf([folderPath,'MC_EKF_Pose'],11,1000);


