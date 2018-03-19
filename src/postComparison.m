%--------------------------------------------------------------------------
%
% File Name:      postComparison.m
% Date Created:   2016/02/20
% Date Modified:  2018/02/13
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Post Process Comparison Trials
%
%--------------------------------------------------------------------------

clean('Figures',0);

%% Parameter Selection

dataOptions = {'Mean','Median'};
dataSel = 'Mean';
simChoose = [1,... % Desired simulation for each control for plotting robot pose
             1];


%% Format Data

% Simulation Trial Folder
[fileName,folderPathOrig,filterIndex] = uigetfile({'information.mat'},'Select Information File');
load([folderPathOrig,fileName]);

% Possible Control Selections
controlConditions = {...
         'Random_Walk',...
         ['Greedy_',s.filterCtrl],...
         ['Rec_Horizon_',s.filterCtrl],...
         ['MCTS_',s.filterCtrl],...
         ['dSigma_dP_',s.filterCtrl],...
         'Approximate_Circle',...
         'Known_Velocities'};

for curCtrl = 1:size(numCtrls,2)
   ctrlInd = numCtrls(curCtrl);
   for simInd = 1:numSims
      
      % Load Data
      clear rob cam est ctrl post trans tInd;
      load([folderPathOrig,'/',controlConditions{ctrlInd},'/','sim_',num2str(simInd),'.mat']);
      
      % Save Specified Simulation Run Data
      if simInd==simChoose(curCtrl)
         comp(ctrlInd).p.pixelTrackPts = p.pixelTrackPts;
         comp(ctrlInd).rob.pose = rob.pose;
         comp(ctrlInd).est = est;
      end
      
      if simInd==1
         GT_total = zeros(3*est.n,cam.tSpanLength,numSims);
         P_total = zeros(3*est.n,cam.tSpanLength,numSims);
         sigma_total = zeros(3*est.n,cam.tSpanLength,numSims);
         ctrlTime_total = zeros(1,cam.tSpanLength,numSims);
      end
      GT_total(:,:,simInd) = est.GT;
      if strcmp(s.filterCtrl,'EKF')
         P_total(:,:,simInd) = est.P_ekf;
         sigma_total(:,:,simInd) = est.sigma_ekf;
      elseif strcmp(s.filterCtrl,'UKF')
         P_total(:,:,simInd) = est.P_ukf;
         sigma_total(:,:,simInd) = est.sigma_ukf;
      end
      ctrlTime_total(:,:,simInd) = p.ctrlTime;
      
   end % end reading numSims simulation runs
   
   % Final Computation on All Simulation Runs
   if strcmp(dataSel,'Mean')
      
      % Compute Mean For All Simulations
      comp(ctrlInd).est.GT = mean(GT_total,3);
      comp(ctrlInd).p.P = mean(P_total,3);
      comp(ctrlInd).p.Perror = mean(abs(P_total-GT_total),3);
      comp(ctrlInd).p.sigma = mean(sigma_total,3);
      comp(ctrlInd).p.ctrlTime = mean(ctrlTime_total,3);
      
      % Compute Lower Bound (-1 std)
      comp(ctrlInd).est.GT_LB = ...
         comp(ctrlInd).est.GT-std(GT_total,0,3);
      comp(ctrlInd).p.P_LB = ...
         comp(ctrlInd).p.P-std(P_total,0,3);
      comp(ctrlInd).p.Perror_LB = ...%comp(ctrlInd).p.Perror
         std(abs(P_total-GT_total),0,3);
      comp(ctrlInd).p.sigma_LB = ...%comp(ctrlInd).p.sigma
         std(sigma_total,0,3);
      comp(ctrlInd).p.ctrlTime_LB = ...
         comp(ctrlInd).p.ctrlTime-std(ctrlTime_total,0,3);
      
      % Compute Upper Bound (+1 std)
      comp(ctrlInd).est.GT_UB = ...
         comp(ctrlInd).est.GT+std(GT_total,0,3);
      comp(ctrlInd).p.P_UB = ...
         comp(ctrlInd).p.P+std(P_total,0,3);
      comp(ctrlInd).p.Perror_UB = ...%comp(ctrlInd).p.Perror+
         std(abs(P_total-GT_total),0,3);
      comp(ctrlInd).p.sigma_UB = ...%comp(ctrlInd).p.sigma+
         std(sigma_total,0,3);
      comp(ctrlInd).p.ctrlTime_UB = ...
         comp(ctrlInd).p.ctrlTime+std(ctrlTime_total,0,3);
      
   elseif strcmp(dataSel,'Median')

      % Compute Median For All Simulations
      comp(ctrlInd).est.GT = median(GT_total,3);
      comp(ctrlInd).p.P = median(P_total,3);
      comp(ctrlInd).p.Perror = median(abs(P_total-GT_total),3);
      comp(ctrlInd).p.sigma = median(sigma_total,3);
      comp(ctrlInd).p.ctrlTime = median(ctrlTime_total,3);
      
      % Compute Lower Bound (25%)
      LB = 25;
      comp(ctrlInd).est.GT_LB = prctile(GT_total,LB,3);
      comp(ctrlInd).p.P_LB = prctile(P_total,LB,3);
      comp(ctrlInd).p.Perror_LB = comp(ctrlInd).p.Perror-prctile(abs(P_total-GT_total),LB,3);
      comp(ctrlInd).p.sigma_LB = comp(ctrlInd).p.sigma-prctile(sigma_total,LB,3);
      comp(ctrlInd).p.ctrlTime_LB = prctile(ctrlTime_total,LB,3);
      
      % Compute Upper Bound (75%)
      UB = 100-LB;
      comp(ctrlInd).est.GT_UB = prctile(GT_total,UB,3);
      comp(ctrlInd).p.P_UB = prctile(P_total,UB,3);
      comp(ctrlInd).p.Perror_UB = prctile(abs(P_total-GT_total),UB,3)-comp(ctrlInd).p.Perror;
      comp(ctrlInd).p.sigma_UB = prctile(sigma_total,UB,3)-comp(ctrlInd).p.sigma;
      comp(ctrlInd).p.ctrlTime_UB = prctile(ctrlTime_total,UB,3);
   end
   
end


%% Control Plot Initialization

% Find Shortest Simulation and Save Plotting Estimation
shortestSim = 1E6;
shortestInd = 0;
for i = numCtrls
   % Shortest Simulation
   ind = comp(i).rob.pose(3,:)~=0;
   testSimLength = sum(ind);
   comp(i).p.tIndFinal = testSimLength;
   if testSimLength<shortestSim
      curSimLength = testSimLength;
      shortestSim = testSimLength;
      shortestInd = i;
   end
   % Camera Indices
   ind = comp(i).est.y(1,:)~=0;
%    ind = comp(i).p.pError(1,:)~=0;
   comp(i).p.cam.tIndFinal = sum(ind);
   
   % Plotting Estimation
   if numSims==1
      if strcmp(s.filterCtrl,'EKF')
         comp(i).p.P = comp(i).est.P_ekf;
         comp(i).p.sigma = comp(i).est.sigma_ekf;
      elseif strcmp(s.filterCtrl,'UKF')
         comp(i).p.P = comp(i).est.P_ukf;
         comp(i).p.sigma = comp(i).est.sigma_ukf;
      end
   end
   
   % Steady State Error:
   error_ss = mean(comp(i).p.Perror(ind));
   disp([controlConditions{i},' steady state estimation error:',sprintf('\n'),num2str(error_ss)]);
   error_ss = mean(comp(i).p.sigma(ind));
   disp([controlConditions{i},' steady state error covariance:',sprintf('\n'),num2str(error_ss)]);
   
end
est.nTemp = comp(1).est.nTemp;

% Simulation Indices
p.tSpan = 0:s.dt:s.tSpan(shortestSim);
p.tSpanLength = length(p.tSpan);
p.plotInd = 1:p.tSpanLength;
p.tInd = shortestSim;

% Estimation Plots
p.cam.tSpan = 0:cam.dt:s.tSpan(shortestSim);
p.cam.tSpanLength = length(p.cam.tSpan);
p.cam.plotInd = 1:p.cam.tSpanLength;
p.cam.tInd = p.cam.tSpanLength(end);

% Plot Continuous Error Bars
p.plotErrorBars = 1;

% Line Style Defintion
p.lines = {':','-.','--','-','-'};
p.n = min(obj.n,length(p.lines));
p.lineWidth = 2;
p.plot.arrLength = 0.5;

% Plotting Color Definition
colors = [1,0,0;
          0.2,1,0;
          0.93, 0.71,0.13;
          0,0,1;
          0.8,0.2,0.8;
          ];


%% Plot Virtual Image
figure(11); clf(11);
hold on;
box on;
axis equal;
axis(cam.plotSize);
% Plot the Feature Trajectory in Virtual Image
for i = numCtrls
   px = reshape(mean(comp(i).p.pixelTrackPts(1,:,1:comp(i).p.cam.tIndFinal),2),1,comp(i).p.cam.tIndFinal);
   py = reshape(mean(comp(i).p.pixelTrackPts(2,:,1:comp(i).p.cam.tIndFinal),2),1,comp(i).p.cam.tIndFinal);
   plot(px,py,'color',colors(i,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{i});
end
% plot(px5,py5,'color',colors(5,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{5}); 
% scatter(px1(1,1),py1(1,1),25,[0,0,0],'fill');
% scatter(px1(1,end),py1(1,end),25,[1,0.5,0],'fill');
% scatter(px2(1,1),py2(1,1),25,[0,0,0],'fill');
% scatter(px2(1,end),py2(1,end),25,[1,0.5,0],'fill');
% scatter(px3(1,1),py3(1,1),25,[0,0,0],'fill');
% scatter(px3(1,end),py3(1,end),25,[1,0.5,0],'fill');
% scatter(px4(1,1),py4(1,1),25,[0,0,0],'fill');
% scatter(px4(1,end),py4(1,end),25,[1,0.5,0],'fill');
hold off;
% title('Virtual Camera Image');
set(gca,'YDir','reverse');
legend('show');
Leg = legend('$\mathrm{Random}: \mathbf{x}$',...
             '$\mathrm{Greedy}: \mathbf{x}$',...
             '$\mathrm{Rec. Horizon}: \mathbf{x}$',...
             '$\mathrm{Gradient}: \mathbf{x}$');
%              '$\mathrm{Circle}: \mathbf{x}$');
%              'Start', 'End');
set(Leg,'Interpreter','Latex');
xHandle = xlabel('x-axis (pixels)'); yHandle = ylabel('y-axis (pixels)');

% World Scene Plotting Volume
obj.plane.max = 50;
p.plotV(1,1) = max([abs(obj.points(1,:)),abs(comp(1).rob.pose(1,:))])+obj.plane.max/5;
p.plotV(2,1) = max([abs(obj.points(2,:)),abs(comp(1).rob.pose(2,:))])+obj.plane.max/5;
p.plotV(3,1) = -max([abs(obj.points(3,:)),abs(comp(1).rob.pose(3,:))])-obj.plane.max/5;
p.plot.area = [-p.plotV(1,1) -p.plotV(2,1);
                  -p.plotV(1,1)  p.plotV(2,1);
                   p.plotV(1,1)  p.plotV(2,1);
                   p.plotV(1,1) -p.plotV(2,1)];


%% Plot Object Scene in World Environment
figure(12); clf(12); 
rotate3d on;
hold on;
box on;
% Plot Object Points
plot3(obj.points(1,:),obj.points(2,:),obj.points(3,:),...
      '.','Color',obj.color.pts);
% Plot Ground Patch
p.groundArea = [p.plot.area];
patch(p.groundArea(:,1),p.groundArea(:,2),obj.color.groundPlane,'EdgeColor','none');
% % Plot Quadrotor and Camera
% for i = 1:4
%    plotQuad(comp(i).rob.pose(:,comp(i).p.tIndFinal),0,0.3,colors(i,:));
%    plotCamera(comp(i).rob.pose(:,comp(i).p.tIndFinal), cam.plotSize, colors(i,:), colors(i,:), p.plot.arrLength, 2)
% end
% Plot Camera Trajectory
h1 = plot3(comp(1).rob.pose(1,1:comp(1).p.tIndFinal),...
      comp(1).rob.pose(2,1:comp(1).p.tIndFinal),...
      comp(1).rob.pose(3,1:comp(1).p.tIndFinal),...
      'Color',colors(1,:),'LineWidth',3);
% h2 = plot3(comp(2).rob.pose(1,1:comp(2).p.tIndFinal),...
%       comp(2).rob.pose(2,1:comp(2).p.tIndFinal),...
%       comp(2).rob.pose(3,1:comp(2).p.tIndFinal),...
%      'Color',colors(2,:),'LineWidth',3);
% % % %TEMPPORATY FIX PLOT%FIX
% % % %THISSSSSSSSS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% % % % fixpose = -([-2.2;1.7;-8.513]-[0;0;-10]); % I don't know
% % % fixpose = -([-1.14;2.47;-8.513]-[0;0;-10]); % works for both
% % % tempose = ...
% % %   rotMat(-pi/6.35,'z')*comp(3).rob.pose(1:3,1:comp(3).p.tIndFinal)+fixpose;
% h3 = plot3(tempose(1,:),tempose(2,:),tempose(3,:),...
%      'Color',colors(3,:),'LineWidth',3);
% h3 = plot3(comp(3).rob.pose(1,1:comp(3).p.tIndFinal),...
%       comp(3).rob.pose(2,1:comp(3).p.tIndFinal),...
%       comp(3).rob.pose(3,1:comp(3).p.tIndFinal),...
%      'Color',colors(3,:),'LineWidth',3);
h4 = plot3(comp(4).rob.pose(1,1:comp(4).p.tIndFinal),...
      comp(4).rob.pose(2,1:comp(4).p.tIndFinal),...
      comp(4).rob.pose(3,1:comp(4).p.tIndFinal),...
      'Color',colors(4,:),'LineWidth',3);
% h5 = plot3(comp(5).rob.pose(1,1:comp(5).p.tIndFinal),...
%       comp(5).rob.pose(2,1:comp(5).p.tIndFinal),...
%       comp(5).rob.pose(3,1:comp(5).p.tIndFinal),...
%       'Color',colors(5,:),'LineWidth',3);
hold off;
% Axes and Legend
xlabel('x-axis (m)'); ylabel('y-axis (m)'); zlabel('z-axis (m)');
axis equal;
axis([-p.plotV(1,1) p.plotV(1,1) -p.plotV(2,1) p.plotV(2,1) p.plotV(3,1) 0]);
view([1,1,1]); % view([-0.3,1,0.4]);
% view([0,0,1]); % top-down view
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');
% Leg = legend([h1,h2,h3,h4,h5],'Random','Greedy','Rec. Horizon','Gradient','Circle',...
%    'Location','NorthEast');
Leg = legend([h1,h2,h3,h4],'Random','Greedy','Rec. Horizon','Gradient',...
   'Location','NorthEast');
set(Leg,'Interpreter','Latex');


%% For Plotting Estimation Error Only
figure(14); clf(14); box on;
ind = linspace(3,3*est.n,est.n);
%%%
% FIXTHIS = 1.0; %EKF
FIXTHIS = 1.2; %UKF
%%%
if obj.n==1
   e1 = (comp(1).p.Perror(ind,:));
   e2 = (comp(2).p.Perror(ind,:));
   e3 = (comp(3).p.Perror(ind,:));
   e4 = (comp(4).p.Perror(ind,:));
   e5 = (comp(5).p.Perror(ind,:));
   e1_UB = (comp(1).p.Perror_UB(ind,:));
   e2_UB = (comp(2).p.Perror_UB(ind,:));
   e3_UB = (comp(3).p.Perror_UB(ind,:));
   e4_UB = (comp(4).p.Perror_UB(ind,:));
   e5_UB = (comp(5).p.Perror_UB(ind,:));
   e1_LB = (comp(1).p.Perror_LB(ind,:));
   e2_LB = (comp(2).p.Perror_LB(ind,:));
   e3_LB = (comp(3).p.Perror_LB(ind,:));
   e4_LB = (comp(4).p.Perror_LB(ind,:));
   e5_LB = (comp(5).p.Perror_LB(ind,:));
else
   e1 = mean(comp(1).p.Perror(ind,:));
%    e2 = mean(comp(2).p.Perror(ind,:));
%    e3 = mean(comp(3).p.Perror(ind,:))/FIXTHIS;
   e4 = mean(comp(4).p.Perror(ind,:));
%    e5 = mean(comp(5).p.Perror(ind,:));
%    e6 = mean(comp(6).p.Perror(ind,:));
   e1_UB = mean(comp(1).p.Perror_UB(ind,:));
%    e2_UB = mean(comp(2).p.Perror_UB(ind,:));
%    e3_UB = mean(comp(3).p.Perror_UB(ind,:))/FIXTHIS;
   e4_UB = mean(comp(4).p.Perror_UB(ind,:));
%    e5_UB = mean(comp(5).p.Perror_UB(ind,:));
%    e6_UB = mean(comp(6).p.Perror_UB(ind,:));
   e1_LB = mean(comp(1).p.Perror_LB(ind,:));
%    e2_LB = mean(comp(2).p.Perror_LB(ind,:));
%    e3_LB = mean(comp(3).p.Perror_LB(ind,:))/FIXTHIS;
   e4_LB = mean(comp(4).p.Perror_LB(ind,:));
%    e5_LB = mean(comp(5).p.Perror_LB(ind,:));
%    e6_LB = mean(comp(6).p.Perror_LB(ind,:));
end
hold on;
clear h1 h2 h3 h4 h5 h6;
if p.plotErrorBars==1
   h1 = shadedErrorBar(p.cam.tSpan,e1(1,1:p.cam.tSpanLength),...
      [e1_UB;e1_LB],... 
      {'Color',colors(1,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{1}},1);
%    h2 = shadedErrorBar(p.cam.tSpan,e2(1,1:p.cam.tSpanLength),...
%       [e2_UB;e2_LB],...
%       {'Color',colors(2,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{2}},1);
%    h3 = shadedErrorBar(p.cam.tSpan,e3(1,1:p.cam.tSpanLength),...
%       [e3_UB;e3_LB],...
%       {'Color',colors(3,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{3}},1);
   h4 = shadedErrorBar(p.cam.tSpan,e4(1,1:p.cam.tSpanLength),...
      [e4_UB;e4_LB],...
      {'Color',colors(4,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{4}},1);
%    h5 = shadedErrorBar(p.cam.tSpan,e5(1,1:p.cam.tSpanLength),...
%       [e5_UB;e5_LB],...
%       {'Color',colors(5,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{5}},1);
%    h6 = shadedErrorBar(p.cam.tSpan,e6(1,1:p.cam.tSpanLength),...
%          [e6_UB;e6_LB],...
%          {'Color',colors(5,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{5}},1);
else
   h1.mainLine = plot(p.cam.tSpan,e1(1,1:p.cam.tSpanLength),...
      'Color',colors(1,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{1});
%    h2.mainLine = plot(p.cam.tSpan,e2(1,1:p.cam.tSpanLength),...
%       'Color',colors(2,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{2});
%    h3.mainLine = plot(p.cam.tSpan,e3(1,1:p.cam.tSpanLength),...
%       'Color',colors(3,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{3});
   h4.mainLine = plot(p.cam.tSpan,e4(1,1:p.cam.tSpanLength),...
      'Color',colors(4,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{4});
%    h5.mainLine = plot(p.cam.tSpan,e5(1,1:p.cam.tSpanLength),...
%       'Color',colors(5,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{5});
%    h6.mainLine = plot(p.cam.tSpan,e6(1,1:p.cam.tSpanLength),...
%       'Color',colors(5,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{5});
end
hold off;
ylabel('Depth Estimation Error: $| s_z - \mu_z | \ (\mathrm{m})$','Interpreter','Latex');
xlabel('Time (s)');
% Leg = legend([h1.mainLine,h2.mainLine,h3.mainLine,h4.mainLine,h5.mainLine,h6.mainLine],...
%    '$\mathrm{Random}: | x_z - \hat{x}_z |$',...
%    '$\mathrm{Greedy}: | x_z - \hat{x}_z |$',...
%    '$\mathrm{Rec. Horizon}: | x_z - \hat{x}_z |$',...
%    '$\mathrm{MCTS-DPW}: | x_z - \hat{x}_z |$',...
%    '$\mathrm{Gradient}: | x_z - \hat{x}_z |$',...
%    '$\mathrm{Circle}: | x_z - \hat{x}_z |$');
Leg = legend([h1.mainLine,h4.mainLine],...
   'Random',...
   'POMCPOWUR');
set(Leg,'Interpreter','Latex','Location','NorthEast');
% axis([0, shortestSim*cam.dt-1, 0, 16]);
axis([0, p.cam.tSpanLength*cam.dt, 0, 25]);


%% State Estimation Error Covariance
figure(15); clf(15); box on;
ind = linspace(3,3*est.n,est.n);
%%%
FIXTHIS = 1.0; %EKF
% FIXTHIS = 1.3; %UKF
%%%
if obj.n==1
   e1 = (comp(1).p.sigma(ind,:));
   e2 = (comp(2).p.sigma(ind,:));
   e3 = (comp(3).p.sigma(ind,:));
   e4 = (comp(4).p.sigma(ind,:));
   e5 = (comp(5).p.sigma(ind,:));
   e1_UB = (comp(1).p.sigma_UB(ind,:));
   e2_UB = (comp(2).p.sigma_UB(ind,:));
   e3_UB = (comp(3).p.sigma_UB(ind,:));
   e4_UB = (comp(4).p.sigma_UB(ind,:));
   e5_UB = (comp(5).p.sigma_UB(ind,:));
   e1_LB = (comp(1).p.sigma_LB(ind,:));
   e2_LB = (comp(2).p.sigma_LB(ind,:));
   e3_LB = (comp(3).p.sigma_LB(ind,:));
   e4_LB = (comp(4).p.sigma_LB(ind,:));
   e5_LB = (comp(5).p.sigma_LB(ind,:));
else
   e1 = mean(comp(1).p.sigma(ind,:));
%    e2 = mean(comp(2).p.sigma(ind,:));
%    e3 = mean(comp(3).p.sigma(ind,:))/FIXTHIS;
%    e3(1) = 100;
   e4 = mean(comp(4).p.sigma(ind,:));
%    e5 = mean(comp(5).p.sigma(ind,:));
%    e6 = mean(comp(6).p.sigma(ind,:));
   e1_UB = mean(comp(1).p.sigma_UB(ind,:));
%    e2_UB = mean(comp(2).p.sigma_UB(ind,:));
%    e3_UB = mean(comp(3).p.sigma_UB(ind,:))/FIXTHIS;
   e4_UB = mean(comp(4).p.sigma_UB(ind,:));
%    e5_UB = mean(comp(5).p.sigma_UB(ind,:));
%    e6_UB = mean(comp(6).p.sigma_UB(ind,:));
   e1_LB = mean(comp(1).p.sigma_LB(ind,:));
%    e2_LB = mean(comp(2).p.sigma_LB(ind,:));
%    e3_LB = mean(comp(3).p.sigma_LB(ind,:))/FIXTHIS;
   e4_LB = mean(comp(4).p.sigma_LB(ind,:));
%    e5_LB = mean(comp(5).p.sigma_LB(ind,:));
%    e6_LB = mean(comp(6).p.sigma_LB(ind,:));
end

hold on;
clear h1 h2 h3 h4 h5;
if p.plotErrorBars==1
   h1 = shadedErrorBar(p.cam.tSpan,e1(1,1:p.cam.tSpanLength),...
      [e1_UB;e1_LB],...
      {'Color',colors(1,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{1}},1);
%    h2 = shadedErrorBar(p.cam.tSpan,e2(1,1:p.cam.tSpanLength),...
%       [e2_UB;e2_LB],...
%       {'Color',colors(2,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{2}},1);
%    h3 = shadedErrorBar(p.cam.tSpan,e3(1,1:p.cam.tSpanLength),...
%       [e3_UB;e3_LB],...
%       {'Color',colors(3,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{3}},1);
   h4 = shadedErrorBar(p.cam.tSpan,e4(1,1:p.cam.tSpanLength),...
      [e4_UB;e4_LB],...
      {'Color',colors(4,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{4}},1);
%    h5 = shadedErrorBar(p.cam.tSpan,e5(1,1:p.cam.tSpanLength),...
%       [e5_UB;e5_LB],...
%       {'Color',colors(5,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{5}},1);
%    h6 = shadedErrorBar(p.cam.tSpan,e6(1,1:p.cam.tSpanLength),...
%       [e6_UB;e6_LB],...
%       {'Color',colors(5,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{5}},1);
else
   h1.mainLine = plot(p.cam.tSpan,e1(1,1:p.cam.tSpanLength),...
      'Color',colors(1,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{1});
%    h2.mainLine = plot(p.cam.tSpan,e2(1,1:p.cam.tSpanLength),...
%       'Color',colors(2,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{2});
%    h3.mainLine = plot(p.cam.tSpan,e3(1,1:p.cam.tSpanLength),...
%       'Color',colors(3,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{3});
   h4.mainLine = plot(p.cam.tSpan,e4(1,1:p.cam.tSpanLength),...
      'Color',colors(4,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{4});
%    h5.mainLine = plot(p.cam.tSpan,e5(1,1:p.cam.tSpanLength),...
%       'Color',colors(5,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{5});
%    h6.mainLine = plot(p.cam.tSpan,e6(1,1:p.cam.tSpanLength),...
%       'Color',colors(5,:),'LineWidth',p.lineWidth,'LineStyle',p.lines{5});
end
hold off;
ylabel('Depth Estimation Error Covariance: $\sigma_{z}^2 \ (\mathrm{m}^2)$','Interpreter','Latex');
xlabel('Time (s)');
% Leg = legend([h1.mainLine,h2.mainLine,h3.mainLine,h4.mainLine,h5.mainLine,h6.mainLine],...
%    '$\mathrm{Random}: \sigma_{z,z}^2$',...
%    '$\mathrm{Greedy}: \sigma_{z,z}^2$',...
%    '$\mathrm{Rec. Horizon}: \sigma_{z,z}^2$',...
%    '$\mathrm{MCTS-DPW}: \sigma_{z,z}^2$',...   
%    '$\mathrm{Gradient}: \sigma_{z,z}^2$',...
%    '$\mathrm{Circle}: \sigma_{z,z}^2$');
Leg = legend([h1.mainLine,h4.mainLine],...
   'Random',...
   'POMCPOWUR');
set(Leg,'Interpreter','Latex');
% axis([0, shortestSim*s.dt-1, 0, 1E2]);
axis([0, p.cam.tSpanLength*cam.dt, 0, 1E2]);

% % Plot Computation Time
% m1 = mean(comp(1).p.ctrlTime(1,1:99));
% m2 = mean(comp(1).p.ctrlTime(1,100:shortestSim));
% test = [comp(1).p.ctrlTime(1,1:99),comp(1).p.ctrlTime(1,100:shortestSim)+(m1-m2)];

%% Control Time

figure(6); clf(6); box on;

% % Plot Loop Time versus Simulation Time
% % semilogy(p.tSpan, comp(1).p.ctrlTime(1,1:shortestSim),'Color',colors(1,:),'LineStyle',p.lines{1},'LineWidth',p.lineWidth);
% semilogy(p.cam.tSpan, comp(1).p.ctrlTime(1,1:p.cam.tSpanLength),'Color',colors(1,:),'LineStyle',p.lines{1},'LineWidth',p.lineWidth);
% hold on;
% semilogy(p.cam.tSpan, comp(2).p.ctrlTime(1,1:p.cam.tSpanLength),'Color',colors(2,:),'LineStyle',p.lines{2},'LineWidth',p.lineWidth);
% semilogy(p.cam.tSpan, comp(3).p.ctrlTime(1,1:p.cam.tSpanLength),'Color',colors(3,:),'LineStyle',p.lines{3},'LineWidth',p.lineWidth);
% semilogy(p.cam.tSpan, comp(4).p.ctrlTime(1,1:p.cam.tSpanLength),'Color',colors(4,:),'LineStyle',p.lines{4},'LineWidth',p.lineWidth);
% hold off;
% ylabel('Control Compuation Loop Time (s)'); xlabel('Simulation Time (s)');
% Leg = legend('$\mathrm{Random}$','$\mathrm{Greedy}$',...
%              '$\mathrm{Active}$','$\mathrm{Circle}$');
% set(Leg,'Interpreter','Latex','Location','Best');
% % axis([0, shortestSim*s.dt-1, 0, 0.1]);
% axis([0, p.cam.tSpanLength*cam.dt, 0, 10]);

% Plot Mean Time in Bar Chart
% c = {'Random','Greedy','Rec. Horizon','Gradient','Circle'};         
% x = 1:5;
% barVals = [mean(comp(1).p.ctrlTime);
%            mean(comp(2).p.ctrlTime);
%            mean(comp(3).p.ctrlTime);
%            mean(comp(4).p.ctrlTime);
%            mean(comp(5).p.ctrlTime)];

c = {'Random','Greedy','Rec. Horizon','Gradient'};         
x = 1:4;
barVals = [mean(comp(1).p.ctrlTime);
           mean(comp(2).p.ctrlTime);
           mean(comp(3).p.ctrlTime);
           mean(comp(4).p.ctrlTime)];
for i = x
   handle(i) = bar(x(i),barVals(i),'BarWidth', 0.7);
   set(handle(i),'FaceColor',colors(i,:));
%    barTopper = sprintf('y(%d) = %.3f', x(i), barVals(i));
% 	text(x(i)-0.2, barVals(i), barTopper, 'FontSize', barFontSize);
%    set(gca,'xticklabel',c{i})
   hold on;
end
set(gca,'yscale','log')
xticklabels(c)         % This will set labels to be used for each tick of the x-axis
xticks(1:1:length(c))  % This will set how many ticks you want on the x-axis. Here, there should be 48 ticks being generated. One for each piece of data you have.
xtickangle(20)                % This will rotate the label so that the labels will not overlap with one another. This is in degrees.
ylabel('Control Compuation Loop Time (s)');

return;
%% Make Pretty Pictures

% Fix Axes Font and Size
figureHandleRange = get(0,'Children')';
set(findall(figureHandleRange,'-property','FontSize'),'FontSize',20);
set(findall(figureHandleRange,'-property','FontName'),'FontName','Times')
axisFontSize = 14;

% % Virtual Image
% figure(1);
% pos = get(1,'Position');
% set(1,'Position',[pos(1:2),560,420]);
% set(gca,'FontSize',axisFontSize);

% Global Pose
figure(12);
pos = get(12,'Position');
set(12,'Position',[pos(1:2),560,420]);
set(gca,'FontSize',axisFontSize);

% % Velocities
% figure(3);
% xlabel('Time (s)');
% pos = get(3,'Position');
% set(3,'Position',[pos(1:2),560,420]);
% set(gca,'FontSize',axisFontSize);

% Covariance
figure(4);
xlabel('Time (s)');
pos = get(4,'Position');
set(4,'Position',[pos(1:2),560,420]);
set(gca,'FontSize',axisFontSize);

% Pz hat
figure(5);
xlabel('Time (s)');
pos = get(5,'Position');
set(5,'Position',[pos(1:2),560,420]);
set(gca,'FontSize',axisFontSize);

% Loop Time
figure(6);
% xlabel('Simulation Time (s)');
pos = get(6,'Position');
set(6,'Position',[pos(1:2),560,420]);
set(gca,'FontSize',axisFontSize);

% % Reconstruction
% figure(7);
% pos = get(7,'Position');
% set(7,'Position',[pos(1:2),560,420]);
% set(gca,'FontSize',axisFontSize)


%% SAVE FIGURES

return
save2pdf([folderPathOrig,'UKF_Perror'],14,1000);
save2pdf([folderPathOrig,'UKF_sigma'],15,1000);

return
figureHandleRange = [14,15];
% figureHandleRange = [14,15,6];
% figureHandleRange = [6];
% figureNames = {'20150225_1','20150225_2','20150225_3','20150225_4','20150225_5'};
extension = 'pdf';
saveFigures('FigureRange',figureHandleRange,...
            'Extension',extension);


%% Specifically for 3D Model Plot

rez=300; %resolution (dpi) of final graphic
f=2; %f is the handle of the figure you want to export
figpos=getpixelposition(f); %dont need to change anything here
resolution=get(0,'ScreenPixelsPerInch'); %dont need to change anything here
set(f,'paperunits','inches','papersize',figpos(3:4)/resolution,'paperposition',[0 0 figpos(3:4)/resolution]); %dont need to change anything here
path='/Users/ericcristofalo/Dropbox/BU/Research/2015_Active_Estimation/3D_Depth_Estimation/Figures'; %the folder where you want to put the file
name='testing.jpg'; %what you want the file to be called
print(f,fullfile(path,name),'-dpng',['-r',num2str(rez)],'-opengl') %save file 

