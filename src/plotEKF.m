%--------------------------------------------------------------------------
%
% File Name:      plotEstimatedPose.m
% Date Created:   2015/03/17
% Date Modified:  2015/09/03
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Script for plotting global robot (camera) pose in 3D
%                 world scene and point estimates.
%
%                 Modified version for AURO v1
%
% Inputs:         
%
% Outputs:        
%
% Example:        
%
%--------------------------------------------------------------------------

% Final Estimated Model Plot
% figure(3); clf(3);
hold on;
rotate3d on;
% box on;
whitebg(obj.color.whitebg);
% set(gcf,'color',[0.8 0.8 0.8]);
% patch(post.plot.area(:,1),post.plot.area(:,2),[0.85 0.90 1],'EdgeColor','none');
plot3(obj.points(1,:),obj.points(2,:),obj.points(3,:),'.','Color',[0.95,0.95,1]);
% plot3(obj.points(1,:),obj.points(2,:),zeros(1,size(obj.points(3,:),2)),'.','Color',[0.8,0.8,1]);

% Plot Point Depth Estimates
R_wc = euler2rot(cam.pose(4:6,tInd));
obj.worldTrackPts = obj.points(1:3,est.ind((est.ind(:,tInd)~=0),tInd));
scatter3(obj.worldTrackPts(1,:),... % plot perfect scene track points
         obj.worldTrackPts(2,:),...
         obj.worldTrackPts(3,:),40,[0,0,1],'Fill');

pointEst = reshape(est.P(1:est.nTemp*3,tInd),3,est.nTemp);
post.camTrackPointEst = R_wc*pointEst+cam.pose(1:3,tInd)*ones(1,est.nTemp);
% post.colorBad = max(max(max(est.Cov(:,:,tInd))));
post.colorBad = max(max(max(est.sigma(:,tInd))));
for i = 1:est.nTemp
   ind = 3*i-2:3*i;
   post.color = max(max(abs(est.sigma(ind,tInd))));
   if post.color>post.colorBad
      post.color = post.colorBad;
   end
   scatter3(post.camTrackPointEst(1,i),... % pixelTrackPts Estimate
            post.camTrackPointEst(2,i),...
            post.camTrackPointEst(3,i),...
            50,...
            [post.color/post.colorBad,1-post.color/post.colorBad,0],...
            'Fill');
end

% % Plot Error Ellipsoid
if ((tInd==1 || tInd==size(cam.pose,2)) && post.plotCovEllipsoids==1) || post.plotEKFellipsoidsOverride==1
   post.n = 50; % resolution of covariance ellipses
   post.nE = post.n/2; % colormap resolution
   for i = 1:est.nTemp
      post.xR = sqrt(est.sigma(3*i-2,tInd));
      post.yR = sqrt(est.sigma(3*i-1,tInd));
      post.zR = sqrt(est.sigma(3*i,tInd));
      [post.xE,post.yE,post.zE] = ellipsoid(pointEst(1,i),pointEst(2,i),pointEst(3,i),...
         post.xR,post.yR,post.zR,post.n);
      post.C = post.zE-ones(size(post.zE))*pointEst(3,i);
      for j = 1:size(post.xE,1)*size(post.xE,2)
         post.X_cam = [post.xE(j);post.yE(j);post.zE(j)];
         post.X_world = R_wc*post.X_cam+cam.pose(1:3,tInd);
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

% Figure Logistics
% title('Estimated Reconstruction');
xlabel('x-axis (m)'); ylabel('y-axis (m)'); zlabel('z-axis (m)');
axis equal;
view([1,1,1]);
% view([1.5,1,1]);
% view([0,0,1]);
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');

% Plot World Coordinate Frame
% plotCoordSys([-post.plotV(1,1)+0.5;-post.plotV(2,1)+0.5;0-0.01;0;0;0], 50, [0,0,0], post.plot.arrLength*1.5, 4)

% Plot Camera Trajectory
% post.camPose = cam.pose;
% post.camPose(:,cam.pose(3,:)==0)=[];
% plot3(post.camPose(1,1:tInd),post.camPose(2,1:tInd),post.camPose(3,1:tInd),'Color',post.trajColor,'LineWidth',3);

% Plot Robot Coordinate Frame
% plotCoordSys(rob.pose(:,tInd), 50, [0,0,1], post.plot.arrLength, 4)

% Plot Camera Coordinate Frame
% plotCoordSys(cam.pose(:,tInd), 50, [0,0,1], post.plot.arrLength, 2)
% plotCamera(cam.pose(:,tInd), cam.plotSize, [0,0,1], [0,0,0], post.plot.arrLength, 2)

% Good Fit Axis
axis([-post.plotV(1,1) post.plotV(1,1) -post.plotV(2,1) post.plotV(2,1) post.plotV(3,1) 0]);
% Standard Axis
% axis([-2.5 2.5 -2.5 2.5 -3 0]);
hold off;

