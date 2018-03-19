%--------------------------------------------------------------------------
%
% File Name:      postProcessing.m
% Date Created:   2015/11/10
% Date Modified:  2016/09/14
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Script for plotting filter estimations
%
% Inputs:         
%
% Outputs:        
%
% Example:        
%
%--------------------------------------------------------------------------

%% Plotting Initialization

% Error Signal
if p.simulationError==1
   warning('Post processing with error. ')
   tInd = tInd-1;
   cam.tInd = cam.tInd-1;
end

% clean('Figures',0);

% Control Plots
p.plotInd = rob.pose(3,:)~=0;
p.tInd = sum(p.plotInd);
p.tSpan = 0:s.dt:s.tSpan(p.tInd);
p.tSpanLength = length(p.tSpan);

% Estimation Plots
p.cam.plotInd = est.y(1,:)~=0;
p.cam.tInd = sum(p.cam.plotInd);
p.cam.tSpan = 0:cam.dt:cam.tSpan(p.cam.tInd);
p.cam.tSpanLength = length(p.cam.tSpan);

% Line Style Defintion
p.lines = {'-','--',':'};
p.n = min(obj.n,length(p.lines));
p.lineWidth = 2;

% Plot Scene
plotEnvironment;


%% Plot Quadrotor Control

figure(11); clf(11); 

% Plot Euler Angle Control
subplot(2,1,1); hold on;
box on;
plot(p.tSpan,rob.pose(4,p.plotInd),'color',[1,0,0],'LineWidth',1);
plot(p.tSpan,rob.pose(5,p.plotInd),'color',[0,1,0],'LineWidth',1);
plot(p.tSpan,rob.pose(6,p.plotInd),'color',[0,0,1],'LineWidth',1);
plot(p.tSpan,ctrl.rob.att.euler_des(1,p.plotInd),'color',[0.6,0,0],'LineWidth',2);
plot(p.tSpan,ctrl.rob.att.euler_des(2,p.plotInd),'color',[0,0.6,0],'LineWidth',2);
plot(p.tSpan,ctrl.rob.att.euler_des(3,p.plotInd),'color',[0,0,0.6],'LineWidth',2);
hold off;
legend('\phi','\theta','\psi','\phi^{des}','\theta^{des}','\psi^{des}');
xlabel('Time (s)');
ylabel('Euler Angles (rad)');
% temp = max(max(abs(rob.pose(4:6,:))))+0.001;
% axis([0,p.tSpan(end),-temp,temp]);

% Plot Velocity Control
subplot(2,1,2); hold on;
box on;
plot(p.tSpan,rob.v(1,p.plotInd),'color',[1,0,0],'LineWidth',1);
plot(p.tSpan,rob.v(2,p.plotInd),'color',[0,1,0],'LineWidth',1);
plot(p.tSpan,rob.v(3,p.plotInd),'color',[0,0,1],'LineWidth',1);
plot(p.tSpan,ctrl.rob.v_des(1,p.plotInd),'color',[0.6,0,0],'LineWidth',2);
plot(p.tSpan,ctrl.rob.v_des(2,p.plotInd),'color',[0,0.6,0],'LineWidth',2);
plot(p.tSpan,ctrl.rob.v_des(3,p.plotInd),'color',[0,0,0.6],'LineWidth',2);
legend('v_x','v_y','v_z','v_x^{des}','v_y^{des}','v_z^{des}');
hold off;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
% temp = max(max(abs(rob.v(1:3,:))))+0.05;
% axis([0,p.tSpan(end),-temp,temp]);


%% Plot Estimation Results

% % Plot Filters
% if any(strcmp(s.estCond,'EKF'))
%    est.P = est.P_ekf;
%    est.sigma = est.sigma_ekf;
%    figure(20); plotEKF;
%    hold on;
%    % Plot Camera Trajectory
%    plot3(cam.pose(1,1:tInd),cam.pose(2,1:tInd),cam.pose(3,1:tInd),'Color',p.trajColor,'LineWidth',3);
%    hold off;
% end
% if any(strcmp(s.estCond,'PF'))
%    figure(21);
%    plotPF;
%    hold on;
%    % Plot Camera Trajectory
%    plot3(cam.pose(1,1:tInd),cam.pose(2,1:tInd),cam.pose(3,1:tInd),'Color',p.trajColor,'LineWidth',3);
%    hold off;
% end
% if any(strcmp(s.estCond,'UKF'))
%    figure(22);
%    plotUKF; hold on;
%    % Plot Camera Trajectory
%    plot3(cam.pose(1,1:tInd),cam.pose(2,1:tInd),cam.pose(3,1:tInd),'Color',p.trajColor,'LineWidth',3);
%    hold off;
% end

% Determine Estimators for Legend
legInd = zeros(1,3);
for i = 1:length(s.estCond)
   test = strcmp({'EKF','UKF','LTVKF'},s.estCond(i));
   legInd(1,test) = find(test==1);
end
legInd(legInd==0)=[];

% P_Z Estimated State Comparison
figure(21); subplot(2,1,1);
hold on;
for i = 1:est.nTemp
   ind = 3*i;
   plot(p.cam.tSpan,est.GT(ind,p.cam.plotInd),...
      'Color',[0,0,0],'LineWidth',p.lineWidth);
   if any(strcmp(s.estCond,'EKF'))
      plot(p.cam.tSpan,est.P_ekf(ind,p.cam.plotInd),...
         'Color',[1,0,0],'LineWidth',p.lineWidth);
   end
   if any(strcmp(s.estCond,'UKF'))
      plot(p.cam.tSpan,est.P_ukf(ind,p.cam.plotInd),...
         'Color',[0,0,1],'LineWidth',p.lineWidth);
   end
   if any(strcmp(s.estCond,'LTVKF'))
      plot(p.cam.tSpan,est.P_ltvkf(ind,p.cam.plotInd),...
         'Color',[0,1,0],'LineWidth',p.lineWidth);
   end
end
title('Estimated Depth');
ylabel('Depth Estimate (m)'); xlabel('Time (s)');
rawLegend = {'EKF: $\hat{P}_z$','UKF: $\hat{P}_z$','LTVKF: $\hat{P}_z$',};
rawLegend = {'$P_z$',rawLegend{legInd}};
Leg = legend(rawLegend);
set(Leg,'Interpreter','Latex');
% grid on;
Z = mean(mean(est.GT(3*(1:est.nTemp),p.cam.plotInd)));
axis([0, p.tSpan(end), Z-10, Z+25]);
% axis([0, p.tSpan(end), 18, 25]);
hold off;

% State Estimation Error Covariance
figure(21); subplot(2,1,2);
hold on;
for i = 1:est.nTemp
   % EKF
   if any(strcmp(s.estCond,'EKF'))
%       plot(p.tSpan,reshape(est.sigma(3*i-2,p.plotInd),1,p.tSpanLength),...
%          'Color',[0.2,0.2,0.7],'LineWidth',p.lineWidth);
%       plot(p.tSpan,reshape(est.sigma(3*i-1,p.plotInd),1,p.tSpanLength),...
%          'Color',[0.2,0.7,0.2],'LineWidth',p.lineWidth);
      plot(p.cam.tSpan,reshape(est.sigma_ekf(3*i,p.cam.plotInd),1,p.cam.tSpanLength),...
         'Color',[1,0,0],'LineWidth',p.lineWidth);
   end
   if any(strcmp(s.estCond,'UKF'))
%       plot(p.tSpan,est.sigma_ukf(1,1:tInd),...
%          'Color',[0,0,1],'LineWidth',p.lineWidth);
%       plot(p.tSpan,est.sigma_ukf(2,1:tInd),...
%          'Color',[0,1,0],'LineWidth',p.lineWidth);
      plot(p.cam.tSpan,reshape(est.sigma_ukf(3*i,p.cam.plotInd),1,p.cam.tSpanLength),...
         'Color',[0,0,1],'LineWidth',p.lineWidth);
   end
   if any(strcmp(s.estCond,'LTVKF'))
%       plot(p.tSpan,est.sigma_ltvkf(1,1:tInd),...
%          'Color',[0,0,1],'LineWidth',p.lineWidth);
%       plot(p.tSpan,est.sigma_ltvkf(2,1:tInd),...
%          'Color',[0,1,0],'LineWidth',p.lineWidth);
      plot(p.cam.tSpan,reshape(est.sigma_ltvkf(3*i,p.cam.plotInd),1,p.cam.tSpanLength),...
         'Color',[0,1,0],'LineWidth',p.lineWidth);
   end
end
hold off;
title('Estimation Error Covariance');
ylabel('Error Covariance (m^2)'); xlabel('Time (s)');
rawLegend = {'EKF: $\sigma_{z,z}^2$', 'UKF: $\sigma_{z,z}^2$', 'LTVKF: $\sigma_{z,z}^2$',};
rawLegend = {rawLegend{legInd}};
Leg = legend(rawLegend);
set(Leg,'Interpreter','Latex');
% grid on;
axis([0, p.tSpan(end), 0, 100]);

% % P_X Estimated State Comparison
% figure(22); subplot(1,2,1);
% hold on;
% for i = 1:est.nTemp
%    ind = 3*i-2;
%    plot(p.cam.tSpan,est.GT(ind,p.cam.plotInd),...
%       'Color',[0,0,0],'LineWidth',p.lineWidth);
%    if any(strcmp(s.estCond,'EKF'))
%       plot(p.cam.tSpan,est.P_ekf(ind,p.cam.plotInd),...
%          'Color',[1,0,0],'LineWidth',p.lineWidth);
%    end
%    if any(strcmp(s.estCond,'UKF'))
%       plot(p.cam.tSpan,est.P_ukf(ind,p.cam.plotInd),...
%          'Color',[0,0,1],'LineWidth',p.lineWidth);
%    end
%    if any(strcmp(s.estCond,'LTVKF'))
%       plot(p.cam.tSpan,est.P_ltvkf(ind,p.cam.plotInd),...
%          'Color',[0,1,0],'LineWidth',p.lineWidth);
%    end
% end
% title('Estimated Depth');
% ylabel('Depth Estimate (m)'); xlabel('Time (s)');
% rawLegend = {'EKF: $\hat{P}_x$', 'LTVKF: $\hat{P}_x$','UKF: $\hat{P}_x$'};
% rawLegend = {'$P_x$',rawLegend{legInd}};
% Leg = legend(rawLegend);
% set(Leg,'Interpreter','Latex');
% % grid on;
% Z = mean(mean(est.GT(3*(1:est.nTemp),p.cam.plotInd)));
% axis([0, p.tSpan(end), -10, 10]);
% % axis([0, p.tSpan(end), 18, 25]);
% hold off;

% % P_Y Estimated State Comparison
% figure(22); subplot(1,2,2);
% hold on;
% for i = 1:est.nTemp
%    ind = 3*i-1;
%    plot(p.cam.tSpan,est.GT(ind,p.cam.plotInd),...
%       'Color',[0,0,0],'LineWidth',p.lineWidth);
%    if any(strcmp(s.estCond,'EKF'))
%       plot(p.cam.tSpan,est.P_ekf(ind,p.cam.plotInd),...
%          'Color',[1,0,0],'LineWidth',p.lineWidth);
%    end
%    if any(strcmp(s.estCond,'UKF'))
%       plot(p.cam.tSpan,est.P_ukf(ind,p.cam.plotInd),...
%          'Color',[0,0,1],'LineWidth',p.lineWidth);
%    end
% 	if any(strcmp(s.estCond,'LTVKF'))
%       plot(p.cam.tSpan,est.P_ltvkf(ind,p.cam.plotInd),...
%          'Color',[0,1,0],'LineWidth',p.lineWidth);
%    end
% end
% title('Estimated Depth');
% ylabel('Depth Estimate (m)'); xlabel('Time (s)');
% rawLegend = {'EKF: $\hat{P}_y$', 'LTVKF: $\hat{P}_y$','UKF: $\hat{P}_y$'};
% rawLegend = {'$P_y$',rawLegend{legInd}};
% Leg = legend(rawLegend);
% set(Leg,'Interpreter','Latex');
% % grid on;
% Z = mean(mean(est.GT(3*(1:est.nTemp),p.cam.plotInd)));
% axis([0, p.tSpan(end), -10, 10]);
% % axis([0, p.tSpan(end), 18, 25]);
% hold off;

% % Filter Time Comparison
% figure(10); subplot(1,5,5);
% % figure(12);
% hold on;
% plot(p.tSpan,p.filterTime(1,p.plotInd).*1000,...
%    'Color',[1,0,0],'LineWidth',p.lineWidth);
% plot(p.tSpan,p.filterTime(2,p.plotInd).*1000,...
%    'Color',[0,1,0],'LineWidth',p.lineWidth);
% plot(p.tSpan,p.filterTime(3,p.plotInd).*1000,...
%    'Color',[0,0,1],'LineWidth',p.lineWidth);
% title('Loop Period');
% ylabel('Loop Period (ms)'); xlabel('Time (s)');
% p.filterTimeMean = mean(p.filterTime,2);
% Leg = legend(['EKF: ',sprintf('%0.05f',p.filterTimeMean(1)),' '],...
%              ['PF: ',sprintf('%0.05f',p.filterTimeMean(2)),' '],...
%              ['UKF: ',sprintf('%0.05f',p.filterTimeMean(3)),' ']);
% set(Leg,'Interpreter','Latex');
% % grid on;
% hold off;

% P_Z Estimated State Comparison
figure(22); subplot(2,1,1);
hold on;
plot(p.cam.tSpan,mean(est.GT(ind,p.cam.plotInd),1),...
   'Color',[0,0,0],'LineWidth',p.lineWidth);
if any(strcmp(s.estCond,'EKF'))
   plot(p.cam.tSpan,mean(est.P_ekf(ind,p.cam.plotInd),1),...
      'Color',[1,0,0],'LineWidth',p.lineWidth);
end
if any(strcmp(s.estCond,'UKF'))
   plot(p.cam.tSpan,mean(est.P_ukf(ind,p.cam.plotInd),1),...
      'Color',[0,0,1],'LineWidth',p.lineWidth);
end
if any(strcmp(s.estCond,'LTVKF'))
   plot(p.cam.tSpan,mean(est.P_ltvkf(ind,p.cam.plotInd),1),...
      'Color',[0,1,0],'LineWidth',p.lineWidth);
end
title('Estimated Mean Depth');
ylabel('Mean Depth Estimate (m)'); xlabel('Time (s)');
rawLegend = {'EKF: $\hat{P}_z$','UKF: $\hat{P}_z$','LTVKF: $\hat{P}_z$'};
rawLegend = {'$P_z$',rawLegend{legInd}};
Leg = legend(rawLegend);
set(Leg,'Interpreter','Latex');
% grid on;
Z = mean(mean(est.GT(3*(1:est.nTemp),p.cam.plotInd)));
axis([0, p.tSpan(end), Z-10, Z+25]);
% axis([0, p.tSpan(end), 18, 25]);
hold off;

% State Estimation Error Covariance
figure(22); subplot(2,1,2);
hold on;
ind = linspace(1,obj.n,obj.n)*3;
% EKF
if any(strcmp(s.estCond,'EKF'))
   plot(p.cam.tSpan,mean(est.sigma_ekf(ind,p.cam.plotInd),1),...
      'Color',[1,0,0],'LineWidth',p.lineWidth);
end
if any(strcmp(s.estCond,'UKF'))
   plot(p.cam.tSpan,mean(est.sigma_ukf(ind,p.cam.plotInd),1),...
      'Color',[0,0,1],'LineWidth',p.lineWidth);
end
if any(strcmp(s.estCond,'LTVKF'))
   plot(p.cam.tSpan,mean(est.sigma_ltvkf(ind,p.cam.plotInd),1),...
      'Color',[0,1,0],'LineWidth',p.lineWidth);
end
hold off;
title('Mean Estimation Error Covariance');
ylabel('Mean Error Covariance (m^2)'); xlabel('Time (s)');
rawLegend = {'EKF: $\sigma_{z,z}^2$', 'UKF: $\sigma_{z,z}^2$', 'LTVKF: $\sigma_{z,z}^2$',};
rawLegend = {rawLegend{legInd}};
Leg = legend(rawLegend);
set(Leg,'Interpreter','Latex');
% grid on;
axis([0, p.tSpan(end), 0, 100]);

return;
figure(10);
pos = get(10,'Position');
set(10,'Position',[pos(1:2),1441,359]);
saveFigures([10],[],'pdf');


%% Final Figure Plotting

if p.plotFinalFigures==1

fontSize = 20;
fontName = 'Timesnewroman';

% Virtual Image
figure(1);
pos = get(1,'Position');
set(1,'Position',[pos(1:2),820,588]);

% Global Pose
figure(2);
pos = get(2,'Position');
set(2,'Position',[pos(1:2),948,801]);

% Estimated Reconstruction
figure(3);
pos = get(3,'Position');
set(3,'Position',[pos(1:2),948,801]);

% Pz hat
figure(4);
xlabel('Time (s)');
pos = get(4,'Position');
set(4,'Position',[pos(1:2),560,420]);

% Covariance
figure(5);
xlabel('Time (s)');
pos = get(5,'Position');
set(5,'Position',[pos(1:2),560,420]);

% Particle Distribution
figure(7);
xlabel('Time (s)');
pos = get(7,'Position');
set(7,'Position',[pos(1:2),560,420]);

% Fix Axes Font and Size
figureHandleRange = get(0,'Children')';
set(findall(figureHandleRange,'-property','FontSize'),'FontSize',20);
set(findall(figureHandleRange,'-property','FontName'),'FontName','Times')

end

return
saveFigures('FigureRange',[1,2],'Extension','pdf');

