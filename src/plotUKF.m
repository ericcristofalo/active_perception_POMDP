%--------------------------------------------------------------------------
%
% File Name:      plotUKF.m
% Date Created:   2015/11/23
% Date Modified:  2016/09/03
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Script for plotting unscented Kalman filter estimation
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
% figure(4); clf(4);
hold on;
rotate3d on;
% box on;
whitebg(obj.color.whitebg);
% set(gcf,'color',[0.8 0.8 0.8]); % set background color
% patch(post.plot.area(:,1),post.plot.area(:,2),[0.85,0.90,1],'EdgeColor','none'); % plot ground
% plot3(obj.points(1,:),obj.points(2,:),obj.points(3,:),'.','Color',[0.95,0.95,1]); % plot full object
% plot3(obj.points(1,:),obj.points(2,:),zeros(1,size(obj.points(3,:),2)),'.','Color',[0.8,0.8,1]);

% Plot Tracked 3D Features
R_wc = euler2rot(cam.pose(4:6,tInd));
obj.worldTrackPts = obj.points(:,post.pixelTrackPts(4,:,tInd)); % corresponding world points
scatter3(obj.worldTrackPts(1,:),... % plot perfect scene track points
         obj.worldTrackPts(2,:),...
         obj.worldTrackPts(3,:),20,[0,0,1],'Fill');

% Plot Feature Estimates
pointEst = reshape(est.P_ukf(1:est.nTemp*3,tInd),3,est.nTemp);
post.camTrackPointEst = R_wc*pointEst+cam.pose(1:3,tInd)*ones(1,est.nTemp);
scatter3(post.camTrackPointEst(1,:),... % pixelTrackPts Estimate
   post.camTrackPointEst(2,:),...
   post.camTrackPointEst(3,:),...
   40,...
   [1,0,0],...
   'Fill');

% Plot Sigma Points
post.temp = R_wc*reshape(ukf.chi(:,:,tInd),3,est.nTemp*ukf.n_a)+cam.pose(1:3,tInd)*ones(1,est.nTemp*ukf.n_a);
plot3(post.temp(1,:),post.temp(2,:),post.temp(3,:),'g.');
% for i = 1:est.nTemp*ukf.n_a
% % %    post.color = est.weights(1,i,tInd)/max(est.weights(1,:,tInd));
% %    post.color = 1;
% %    plot3(post.temp(1,i),... % pixelTrackPts Estimate
% %       post.temp(2,i),...
% %       post.temp(3,i),...
% %       '.',...
% %       'Color',[1-post.color,post.color,0.25]);
%    scatter3(post.temp(1,i),... % pixelTrackPts Estimate
%             post.temp(2,i),...
%             post.temp(3,i),...
%             100,...
%             [1,0,0],...
%             'Fill');
% end

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

% % Plot Camera Coordinate Frame
% plotCoordSys(cam.pose(:,tInd), 50, [0,0,1], post.plot.arrLength, 2)
% plotCamera(cam.pose(:,tInd), cam.plotSize, [0,0,1], [0,0,0], post.plot.arrLength, 2)

% Good Fit Axis
axis([-post.plotV(1,1) post.plotV(1,1) -post.plotV(2,1) post.plotV(2,1) post.plotV(3,1) 0]);
% Standard Axis
% axis([-2.5 2.5 -2.5 2.5 -3 0]);
hold off;


%% Save Image

return
fileName = [num2str(tInd),'.jpg'];
filePath = [pwd,'/Figures/PF_Comparison/test/'];

rez = 100; %resolution (dpi) of final graphic
f = 3; %f is the handle of the figure you want to export
figpos = getpixelposition(f); %dont need to change anything here
resolution = get(0,'ScreenPixelsPerInch'); %dont need to change anything here
set(f,'paperunits','inches','papersize',figpos(3:4)/resolution,'paperposition',[0 0 figpos(3:4)/resolution]); %dont need to change anything here
print(f,fullfile(filePath,fileName),'-dpng',['-r',num2str(rez)],'-opengl') %save file 

% saveas(3,[pwd,'/Figures/PF_Comparison/',fileName]);


%% Display Particles
return
tPlot = tInd;
% tPlot = 1198;

figure(10); clf(10); hold on;
% Plot Particles from Filter
R_wc = euler2rot(cam.pose(4:6,tPlot));
post.temp = R_wc*est.x_(:,:,tPlot)+cam.pose(1:3,tPlot)*ones(1,est.N);
for i = 1:est.N
   post.color = est.w(i,tPlot)/max(est.w(:,tPlot));
   post.size = max(ceil(150*post.color),1);
   scatter3(post.temp(1,i),... % pixelTrackPts Estimate
            post.temp(2,i),...
            post.temp(3,i),...
            post.size,...
            [1-post.color,post.color,0.5],...
            'Fill');
%    plot3(post.temp(1,i),... % pixelTrackPts Estimate
%          post.temp(2,i),...
%          post.temp(3,i),...
%          '.',...
%          'Color',[1-post.color,post.color,0.5]);
end
xlabel('x-axis (m)'); ylabel('y-axis (m)'); zlabel('z-axis (m)');
axis equal;
view([1,1,1]);
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');
hold off;

