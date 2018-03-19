%--------------------------------------------------------------------------
%
% File Name:      quadPostProcessing.m
% Date Created:   2016/08/29
% Date Modified:  2016/08/29
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Quadrotor simulation post processing script
%
%
%--------------------------------------------------------------------------


%% Post Processing

p.plotInd = q.x(3,:)~=0;
p.tSpan = s.tSpan(p.plotInd);
p.q.v = q.v(:,p.plotInd);
p.c.v_des = c.v_des(:,p.plotInd);
p.q.euler = q.euler(:,p.plotInd);
p.c.att.euler_des = c.att.euler_des(:,p.plotInd);
p.c.tau = c.tau(:,p.plotInd);

figure(2); clf(2); hold on;
box on;
plot(p.tSpan,p.q.euler(1,:),'color',[1,0,0],'LineWidth',1);
plot(p.tSpan,p.q.euler(2,:),'color',[0,1,0],'LineWidth',1);
plot(p.tSpan,p.q.euler(3,:),'color',[0,0,1],'LineWidth',1);
plot(p.tSpan,p.c.att.euler_des(1,:),'color',[0.6,0,0],'LineWidth',2);
plot(p.tSpan,p.c.att.euler_des(2,:),'color',[0,0.6,0],'LineWidth',2);
plot(p.tSpan,p.c.att.euler_des(3,:),'color',[0,0,0.6],'LineWidth',2);
hold off;
xlabel('Time (s)');
ylabel('Euler Angles (rad)');
% axis([0,p.tSpan(end),-pi/8,pi/8]);

figure(3); clf(3); hold on;
box on;
plot(p.tSpan,p.q.v(1,:),'color',[1,0,0],'LineWidth',1);
plot(p.tSpan,p.q.v(2,:),'color',[0,1,0],'LineWidth',1);
plot(p.tSpan,p.q.v(3,:),'color',[0,0,1],'LineWidth',1);
plot(p.tSpan,p.c.v_des(1,:),'color',[0.6,0,0],'LineWidth',2);
plot(p.tSpan,p.c.v_des(2,:),'color',[0,0.6,0],'LineWidth',2);
plot(p.tSpan,p.c.v_des(3,:),'color',[0,0,0.6],'LineWidth',2);
hold off;
xlabel('Time (s)');
ylabel('Velocity (m/s)');

figure(4); clf(4); hold on;
box on;
plot(p.tSpan,p.c.tau(1,:),'color',[1,0,0],'LineWidth',1);
plot(p.tSpan,p.c.tau(2,:),'color',[0,1,0],'LineWidth',1);
plot(p.tSpan,p.c.tau(3,:),'color',[0,0,1],'LineWidth',1);
hold off;
xlabel('Time (s)');
ylabel('Applied Torque (Nm)');












