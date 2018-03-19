%--------------------------------------------------------------------------
%
% File Name:      plotEnvironment.m
% Date Created:   2016/09/02
% Date Modified:  2017/10/15
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Plot Quadrotor and 3D Model    
%
%--------------------------------------------------------------------------

%% Plot Virtual Image

% Plot Camera After New Image is Acquired
if cam.plotInd~=cam.tInd

   if (p.movie==1)
      figure(1); clf(1); subplot(2,4,[1:2,5:6]); hold on;
   else
      figure(1); clf(1); hold on;
   end
   
   % Plot Individual Imaged Feature Trajectories Over Time
   for i = 1:est.n
      ind = reshape(p.pixelTrackPts(1,i,1:cam.tInd)~=0,1,cam.tInd);
      plot(reshape(p.pixelTrackPts(1,i,ind),1,sum(ind)),...
           reshape(p.pixelTrackPts(2,i,ind),1,sum(ind)),...
           'Color',[0.9,0.95,1]);
   end
   
   % Plot Image Feature Mean Trajectories Using Projected Estimation
   if cam.tInd>1
      % Plot the Mean Estimated Feature Trajectory in Virtual Image
      p.pixelTrackPtsMean = zeros(2,cam.tInd);
      for i = 1:cam.tInd
         if any(strcmp(s.estCond,'EKF'))
            temp = mean(reshape(est.P_ekf(:,i),3,est.n),2);
         elseif any(strcmp(s.estCond,'UKF'))
            temp = mean(reshape(est.P_ukf(:,i),3,est.n),2);
         elseif any(strcmp(s.estCond,'LTVKF'))
            temp = mean(reshape(est.P_ltvkf(:,i),3,est.n),2);
         end
         temp = cam.K*temp./temp(3);
         p.pixelTrackPtsMean(:,i) = temp(1:2,1);
      end
      plot(p.pixelTrackPtsMean(1,:),p.pixelTrackPtsMean(2,:),'b','LineWidth',2);
      
%       % Plot the Mean Visible Feature Trajectory in Virtual Image
%       p.px = reshape(mean(p.pixelTrackPts(1,:,1:cam.tInd),2),1,cam.tInd);
%       p.py = reshape(mean(p.pixelTrackPts(2,:,1:cam.tInd),2),1,cam.tInd);
%       plot(p.px,p.py,'b','LineWidth',2);
   end
   
   % Plot Image's Current Feature Positions
   plotImage(obj.pixelPts,...
             p.pixelTrackPts(:,p.pixelTrackPts(1,:,cam.tInd)~=0,cam.tInd),...
             cam.plotSize,...
             obj.color.pts,...
             obj.color.trackPts);

   hold off;
   if (p.movie==1)
      set(gca,'xtick',[]);
      set(gca,'ytick',[]);
      set(gcf,'color','w');
   else
      xHandle = xlabel('x-axis (pixels)'); yHandle = ylabel('y-axis (pixels)');
   end
   % title('Virtual Camera Image');
   set(gca,'YDir','reverse');

end
cam.plotInd = cam.tInd;


%% Plot 3D Environment

if (p.movie==1)
   figure(1); subplot(2,4,[3:4,7:8]); hold on;
else
   figure(2); clf(2); hold on;
end
hold on;
rotate3d on;
box on; grid on;
whitebg(obj.color.whitebg); % background color

% STL Face Properties
if obj.sample==0
   obj.P = patch('faces',obj.f,'vertices',obj.v(1:3,:)');
   set(obj.P,'FaceColor',obj.color.face,'FaceAlpha',0.5);
   set(obj.P,'EdgeColor',obj.color.edge,'LineWidth',1);
end

% Plot Model
plot3(obj.points(1,:),obj.points(2,:),obj.points(3,:),...
      '.','Color',obj.color.pts);
obj.worldTrackPts = obj.points(:,p.pixelTrackPts(4,:,cam.tInd)); % corresponding world points
scatter3(obj.worldTrackPts(1,:),...
         obj.worldTrackPts(2,:),...
         obj.worldTrackPts(3,:),30,obj.color.trackPts,'Fill');

% Plot Robot
plotQuad(rob.pose(:,tInd),2*0.5,2*0.3,[0,0,1]);

% Plot Trajectory
plot3(rob.pose(1,1:tInd),rob.pose(2,1:tInd),rob.pose(3,1:tInd),...
      'Color',[1,0.5,0.25],'LineWidth',2);

% Update Camera's Tranformation For Virtual Image Acquisition
cam.pose(1:3,1) = rob.pose(1:3,tInd)+trans.R_wr*(cam.offset(1:3,1));
trans.R_wc = trans.R_wr*trans.R_rc; % camera points to robot, then robot to world points
trans.R_cw = inv(trans.R_wc);
cam.pose(4:6,1) = rot2euler(trans.R_wc);
trans.camTrans = [trans.R_cw,-trans.R_cw*rob.pose(1:3,tInd); ...
   0,0,0,1]; % using inverse of camera to world (world to camera transformation)

% Plot Camera
plotCoordSys(cam.pose(:,1), '', 50, [0,0,1], 1, p.arrowLength, 2);
plotCamera(cam.pose(:,1), cam.plotSize, [0,0,1], [0,0,0], p.arrowLength, 2);

% Plot Error Ellipsoid
if ( (cam.tInd~=1 && p.plotCovEllipsoids==1) || (cam.tInd~=1 && p.plotEKFellipsoidsOverride==1) )
   post.n = 50; % resolution of covariance ellipses
   post.nE = post.n/2; % colormap resolution
   SigmaTemp = [];
   if any(strcmp(s.estCond,'EKF'))
      SigmaTemp = mean(reshape(est.sigma_ekf(:,cam.tInd),3,est.n),3);
   elseif any(strcmp(s.estCond,'UKF'))
      SigmaTemp = mean(reshape(est.sigma_ukf(:,cam.tInd),3,est.n),3);
   elseif any(strcmp(s.estCond,'LTVKF'))
      SigmaTemp = mean(reshape(est.sigma_ltvkf(:,cam.tInd),3,est.n),3);
   end
   for i = 1:est.n
      post.xR = sqrt(SigmaTemp(3*i-2));
      post.yR = sqrt(SigmaTemp(3*i-1));
      post.zR = sqrt(SigmaTemp(3*i-0));
      pointEst = est.P_ekf((3*i-2):(3*i),cam.tInd);
      [post.xE,post.yE,post.zE] = ellipsoid(pointEst(1,1),pointEst(2,1),pointEst(3,1),...
         post.xR,post.yR,post.zR,post.n);
      post.C = post.zE-ones(size(post.zE))*pointEst(3,1);
      for j = 1:size(post.xE,1)*size(post.xE,2)
         post.X_cam = [post.xE(j);post.yE(j);post.zE(j)];
         post.X_world = trans.R_wc*post.X_cam+cam.pose(1:3,1);
         post.xE(j) = post.X_world(1);
         post.yE(j) = post.X_world(2);
         post.zE(j) = post.X_world(3);
      end
      post.S = surf(post.xE, post.yE, post.zE);
      post.colormap = [linspace(1,0,post.nE)',linspace(0,1,post.nE)',linspace(0,0.3,post.nE)';
      linspace(0,1,post.nE)',linspace(1,0,post.nE)',linspace(0.3,0,post.nE)'];
      colormap(post.colormap);
      set(post.S,'CData',post.C);
      set(post.S,'EdgeColor','none','FaceAlpha',0.2);
   end
end

% Plot Axes Labels
% title('Global Coordinate System');
if (p.movie==1)
   set(gca,'visible','off');
   set(gcf,'color','w');
else
   xlabel('x-axis (m)'); ylabel('y-axis (m)'); zlabel('z-axis (m)');
end
axis equal;
view([1,0.6,0.6]); % good angle
% view([1,1,1]);
% view([-0.3,1,0.4]);
% view([0,0,1]); % top-down view
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');

% Compute New Axes Volume
p.plotVolCur = zeros(1,6);
p.checkPts = rob.pose(1,:)~=0;
p.plotVolCur(1,1) = min([(obj.points(1,:)),(rob.pose(1,p.checkPts))])+p.plotVol(1,1);
p.plotVolCur(1,2) = max([(obj.points(1,:)),(rob.pose(1,p.checkPts))])+p.plotVol(1,2);
p.plotVolCur(1,3) = min([(obj.points(2,:)),(rob.pose(2,p.checkPts))])+p.plotVol(1,3);
p.plotVolCur(1,4) = max([(obj.points(2,:)),(rob.pose(2,p.checkPts))])+p.plotVol(1,4);
p.plotVolCur(1,5) = min([(obj.points(3,:)),(rob.pose(3,p.checkPts))])+p.plotVol(1,5);
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


%% SAVE FIGURES

return
FigureRange = [2];
figure(2);
set(gca,'visible','off');
set(gcf,'color','w');
% figureHandleRange = [];
% figureNames = {'20150225_1','20150225_2','20150225_3','20150225_4','20150225_5'};
extension = 'png';
saveFigures('FigureRange',FigureRange,'Transparency',1,'extension',extension);

