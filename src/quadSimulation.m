%--------------------------------------------------------------------------
%
% File Name:      quadSimulation.m
% Date Created:   2016/09/02
% Date Modified:  2016/09/02
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Quadrotor simulation
%
%--------------------------------------------------------------------------

clean('Figures',0);

%% Initialization

% Simulation
s.t0 = 0;
s.tf = 1; % seconds
s.dt = 0.005; % seconds
s.tSpan = s.t0:s.dt:s.tf;
s.tSpanLength = length(s.tSpan);
s.tSpanInd = 1:1:s.tSpanLength;

% Dynamics
q.m = 0.9; % kg
q.J = ... % kg*m^2
   [4.8077196e+03/1000000, -1.7793851e-01/1000000, 8.0944927e+01/1000000;
    -1.7793851e-01/1000000, 8.2709448e+03/1000000, -2.3586788e+00/1000000;
    8.0944927e+01/1000000, -2.3586788e+00/1000000, 4.8175937e+03/1000000];
q.J_inv = inv(q.J);
g = [0;0;9.81]; % m/s^2

% States
q.x = zeros(3,s.tSpanLength);
q.x(:,1) = [0;0;-2.0];
q.xPrev = q.x(:,1);

q.v = zeros(3,s.tSpanLength);
q.v(:,1) = [0;0;0];

q.a = zeros(3,s.tSpanLength);
q.a(:,1) = [0;0;0];

q.euler = zeros(3,s.tSpanLength);
q.euler(:,1) = [0;0;0];
q.eulerPrev = q.euler(:,1);

q.R_wr = euler2rot(q.euler(:,1));
q.R_wrDot = zeros(3,3);

q.omega = zeros(3,s.tSpanLength);
q.omega(:,1) = [0;0;0];

q.Omega = skewSymMat(q.omega(:,1));

q.omegaDot = zeros(3,s.tSpanLength);
q.omegaDot(:,1) = [0;0;0];

% Control Inputs
c.v_des = zeros(3,s.tSpanLength);
c.vErr = zeros(3,1);
c.aErr = zeros(3,1);
c.yaw_des(1,1) = q.euler(3);
c.yawCond = 0;
c.z_des = -2.0;
c.att.euler_des = zeros(3,s.tSpanLength);
c.tau = zeros(3,s.tSpanLength);

% Plotting
figure(1);
p.plotInd = 0;
p.plotVal = 1;
p.plotL = 1.0;
p.plotVol = [-p.plotL,p.plotL,...
                -p.plotL,p.plotL,...
                -p.plotL,0];

% %% Joystick use: >> instrfind
% serPort = serial('/dev/ttys1', 'Baudrate', 115200);
% fopen(serPort);
% joy = vrjoystick(id)


%% Simulation

for tInd = s.tSpanInd
   
   %-----------------------------------------------------------------------
   
   % Skip Initial Conditions
   if tInd<2 % skip initial conditions
      continue;
   end
   
   % Crash Indicator
   if q.x(3,tInd-1)>0
      disp('Crash!');
      break;
   end
   
   % Time
   t = s.tSpan(tInd);
   
   %-----------------------------------------------------------------------
   
   % Desired Flat Body Frame Velocity (m/s)
   c.v_des(:,tInd) = [0; % for gain tunning
                      0.1;
                      0];
%    c.v_des(:,tInd) = [1*sin(t/1);
%                       1*cos(t/1);
%                       0];
%    % Desired Global Yaw
%    if c.yawCond==0 && c.yaw_des>=pi
%       c.yaw_des = -pi;
%       c.yawCond = 1;
%    end
%    if c.yawCond==1 && c.yaw_des>0
%       c.yaw_des = 0;
%       c.yawCond = 0;
%    end
%    c.yaw_des = c.yaw_des+0.005;

   % Closed-loop Attitude Control
   % UPenn: Trajectory Generation and Control for Precise Aggressive Maneuvers with Quadrotors
   % Set Control Gains on First Iteration
   if tInd==2
      % Velocity Controller [100;1;1]
      c.kp = -30*diag([1;1;1]);
      c.kd = -0.4*diag([1;1;1]);
      c.ki = -0*diag([1;1;1]);
      % Attitude Controller [1000;10000]
      c.att.kp = -0*diag([1;1;0.01]);
      
      c.att.kd = -1*diag([1;1;1]);
   end
   % Velocity Control
   c.vErrPrev = c.vErr;
	c.vErr = [q.v(1,tInd-1)-c.v_des(1,tInd); 
             q.v(2,tInd-1)-c.v_des(2,tInd); 
             (q.x(3,tInd-1)-c.z_des)/s.dt];
   c.a_des = c.kp*c.vErr + ...
             c.ki*(c.vErr+c.vErrPrev)*s.dt + ...
             c.kd*(c.vErr-c.vErrPrev)/s.dt;
   % Desired Euler Angles
   c.att.euler_des(:,tInd) = -1/g(3)*...
      [c.a_des(1,1)*sin(q.euler(3,tInd-1))-c.a_des(2,1)*cos(q.euler(3,tInd-1));
       c.a_des(1,1)*cos(q.euler(3,tInd-1))+c.a_des(2,1)*sin(q.euler(3,tInd-1));
       -g(3)*c.yaw_des];
   % Cap Desired Euler Angles
   testEuler = c.att.euler_des(1:2,tInd);
   ind = abs(testEuler)>0.2;
   if any(ind)
      c.att.euler_des(ind,tInd) = 0.2*(testEuler(ind)./abs(testEuler(ind)));
   end
   % Attitude Control
   c.att.eulerErr = q.euler(:,tInd-1)-c.att.euler_des(:,tInd);
   c.att.omega_des = -(q.euler(:,tInd-1)-c.att.euler_des(:,tInd))/s.dt;
   c.att.omegaErr = (q.omega(:,tInd-1)-c.att.omega_des);
   c.tau(:,tInd) = c.att.kp*c.att.eulerErr + c.att.kd*(c.att.omegaErr);
%    % Cap Applied Torque
%    testTau = c.tau(:,tInd);
%    ind = abs(testTau)>10000;
%    if any(ind)
%       disp(['Capping Tau at time: ',num2str(t)]);
%       c.tau(ind,tInd) = 10000*(testTau(ind)./abs(testTau(ind)));
%    end
   
   % Force Control
%    % Compute F as a Function of Orientation
%    c.F = q.m*g(3,1)/(cos(q.euler(1,tInd))*cos(q.euler(2,tInd)));
   % Compute F from UPenn Control
   c.F = -q.m*c.a_des(3,1);
   % Thrust During Unstable Flight
   if c.F<0
      c.F = 0;
   elseif c.F>20
      c.F = 20;
   end
   
   % Display Outputs
   if p.plotInd==p.plotVal-1
%       disp('accelerations: ');
%       [q.a(2,1);q.a(1,1);0]
%       disp('v error: ');
%       c.vErr
%       disp('a error: ');
%       c.aErr
%       disp('tau(3): ');
%       c.tau(3)
%       disp('euler: ');
%       q.euler
%       disp('yaw desired: ');
%       c.yaw_des
   end
   
   %-----------------------------------------------------------------------
   
   % Update Orientation
   q.omegaDot(:,tInd) = q.J_inv*c.tau(:,tInd)-q.J_inv*q.Omega*q.J*q.omega(:,tInd-1);
   q.omega(:,tInd) = q.omegaDot(:,tInd)*s.dt+q.omega(:,tInd-1);
   q.Omega = skewSymMat(q.omega(:,tInd));
   R_r1r2 = expm(q.Omega*s.dt);
   q.R_wr = q.R_wr*R_r1r2;
   q.euler(:,tInd) = rot2euler(q.R_wr);
%    q.omegaDot = q.J_inv*c.tau-q.J_inv*q.Omega*q.J*q.omega;
%    q.omega = q.omegaDot*s.dt+q.omega;
%    q.Omega = skewSymMat(q.omega);
%    R_r1r2 = expm(q.Omega*s.dt);
%    q.R_wr = q.R_wr*R_r1r2;
   
   % Update Position
   q.a(:,tInd) = g-(c.F/q.m)*q.R_wr*[0;0;1];
   q.v(:,tInd) = q.a(:,tInd)*s.dt+q.v(:,tInd-1);
   q.x(:,tInd) = q.v(:,tInd)*s.dt+q.x(:,tInd-1);
%    q.a = g-(c.F/q.m)*q.R_wr*[0;0;1];
%    q.v = q.a*s.dt+q.v;
%    q.x = q.v*s.dt+q.x;
   
   %-----------------------------------------------------------------------

   % Draw Robot
   p.plotInd = p.plotInd+1;
   if p.plotInd==p.plotVal 
      p.plotInd=0;
      
      % Display
%       disp(['Force: ',num2str(c.F)]);
%       fprintf(['Velocity: \n',num2str(q.v(1,1)),'\n',...
%                            num2str(q.v(2,1)),'\n',...
%                            num2str(q.v(3,1)),'\n\n'])
      
      % Figure Window
      figure(1); clf(1); hold on;
      rotate3d on;
      box on; grid on;
      % Plot Quadrotor
      plotQuad([q.x(:,tInd); rot2euler(q.R_wr)],0.2,0.1,[0,0,1]);
      % Labels
      xlabel('x-axis (m)'); ylabel('y-axis (m)'); zlabel('z-axis (m)');
      axis equal;
      set(gca,'XDir','reverse');
      set(gca,'ZDir','reverse');
      % Compute New Plotting Axes
      p.plotVolCur = [q.x(1,tInd)-p.plotL,q.x(1,tInd)+p.plotL,...
                         q.x(2,tInd)-p.plotL,q.x(2,tInd)+p.plotL,...
                         q.x(3,tInd)-p.plotL,0];
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
      view([1,0.6,0.6]); % good angle
      % Plot Ground
      p.plotArea = [p.plotVolTemp(1),p.plotVolTemp(3);
                       p.plotVolTemp(2),p.plotVolTemp(3);
                       p.plotVolTemp(2),p.plotVolTemp(4);
                       p.plotVolTemp(1),p.plotVolTemp(4)];
      obj.color.groundPlane = [0.55 0.77 0.55]; % grass green
      patch(p.plotArea(:,1),p.plotArea(:,2),obj.color.groundPlane,'EdgeColor','none');
      % Plot Description
      descr = {['Time: ',num2str(round(t)),' seconds'];
               'Quadrotor Pose';
               ['x: ',num2str(q.x(1,tInd))];
               ['y: ',num2str(q.x(2,tInd))];
               ['z: ',num2str(q.x(3,tInd))];
               ['\phi: ',num2str(q.euler(1,tInd))];
               ['\theta: ',num2str(q.euler(2,tInd))];
               ['\psi: ',num2str(q.euler(3,tInd))];
               'Velocity';
               ['v_x: ',num2str(q.v(1,tInd))];
               ['v_y: ',num2str(q.v(2,tInd))];
               ['v_z: ',num2str(q.v(3,tInd))];
               'Force';
               ['F: ', num2str(c.F)];
               };
      text(4.0,0.0,descr)
   end
   
end

quadPostProcessing;

