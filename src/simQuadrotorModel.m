%--------------------------------------------------------------------------
%
% File Name:      auroQuadrotorModel.m
% Date Created:   2016/09/02
% Date Modified:  2016/09/06
%
% Author:         Eric Cristofalo
% Contact:        erictrl.rob.cristofalo@gmail.com
%
% Description:    Move Quadrotor in 3D Environment    
%
%--------------------------------------------------------------------------


%% Compute Quadrotor Control

% % Desired Global Yaw
% if ctrl.rob.yawCond==0 && ctrl.rob.yaw_des>=pi
%    ctrl.rob.yaw_des = -pi;
%    ctrl.rob.yawCond = 1;
% end
% if ctrl.rob.yawCond==1 && ctrl.rob.yaw_des>0
%    ctrl.rob.yaw_des = 0;
%    ctrl.rob.yawCond = 0;
% end
% ctrl.rob.yaw_des = ctrl.rob.yaw_des+0.005;

% Closed-loop Attitude Control
% UPenn: Trajectory Generation and Control for Precise Aggressive 
% Maneuvers with Quadrotors
% Set Control Gains on First Iteration
if tInd==2
   % Velocity Controller [50;0.1;0]
   ctrl.rob.kp = -50*diag([1;1;1]);
   ctrl.rob.kd = -0.1*diag([1;1;1]);
   ctrl.rob.ki = -0*diag([1;1;1]);
   % Attitude Controller [0.1;0.8]
   ctrl.rob.att.kp = -0.1*diag([1;1;1]);
   ctrl.rob.att.kd = -0.8*diag([1;1;1]);   
end

% % Altitude Control
% if strcmp(s.estCond,'EKF') && cam.tInd>1
%    ctrl.rob.k_z = -1/(mean(est.sigma_ekf(3*(1:est.n),cam.tInd-1)));
%    ctrl.rob.alt = -mean(est.P_ekf(3*(1:est.n),cam.tInd));
%    ctrl.z_error = ctrl.rob.k_z*(ctrl.rob.alt-ctrl.rob.z_des)/1;
% elseif strcmp(s.estCond,'UKF') && cam.tInd>1
%    ctrl.rob.k_z = -1/(mean(est.sigma_ukf(3*(1:est.n),cam.tInd-1)));
%    ctrl.rob.alt = -mean(est.P_ukf(3*(1:est.n),cam.tInd));
%    ctrl.z_error = ctrl.rob.k_z*(ctrl.rob.alt-ctrl.rob.z_des)/1;
% else
%    ctrl.rob.kp(3,3) = -1;
%    ctrl.rob.kd(3,3) = -0;
%    ctrl.rob.ki(3,3) = -0;
%    ctrl.rob.k_z = 1;
%    ctrl.z_error = -rob.m*rob.gravity(3,1)/...
%                   (cos(rob.pose(3,tInd-1))*cos(rob.pose(2,tInd-1)));
% end
% Ground Truth Altitude Control
ctrl.rob.kp(3,3) = -4000; % keep higher for lower overshoot
ctrl.rob.kd(3,3) = -50;
ctrl.rob.ki(3,3) = -0;
ctrl.z_error = (rob.pose(3,tInd-1)-ctrl.rob.z_des); % perfect altitude
% % Estimated Altitude Control
% ctrl.rob.kp(3,3) = -1; % keep higher for lower overshoot
% ctrl.rob.kd(3,3) = -0;
% ctrl.rob.ki(3,3) = -0;
% if strcmp(s.estCond,'EKF')
%    ctrl.z_error = (-mean(est.P_ekf(3*(1:10),cam.tInd))-ctrl.rob.z_des);
% elseif strcmp(s.estCond,'UKF')
%    ctrl.z_error = (-mean(est.P_ukf(3*(1:10),cam.tInd))-ctrl.rob.z_des);
% end

% Velocity Control
ctrl.rob.vErrPrev = ctrl.rob.vErr;
ctrl.rob.v_des(:,tInd) = ctrl.nu(:,cam.tInd);
ctrl.rob.vErr = [rob.v(1,tInd-1)-ctrl.rob.v_des(1,tInd);
                 rob.v(2,tInd-1)-ctrl.rob.v_des(2,tInd);
                 ctrl.z_error];
ctrl.rob.a_des = ctrl.rob.kp*ctrl.rob.vErr + ...
                 ctrl.rob.ki*(ctrl.rob.vErr+ctrl.rob.vErrPrev)*s.dt + ...
                 ctrl.rob.kd*(ctrl.rob.vErr-ctrl.rob.vErrPrev)/s.dt;
% Desired Euler Angles
ctrl.rob.att.euler_des(:,tInd) = -1/rob.gravity(3)*...
   [ctrl.rob.a_des(1,1)*sin(rob.pose(6,tInd-1))-...
         ctrl.rob.a_des(2,1)*cos(rob.pose(6,tInd-1));
    ctrl.rob.a_des(1,1)*cos(rob.pose(6,tInd-1))+...
         ctrl.rob.a_des(2,1)*sin(rob.pose(6,tInd-1));
    -rob.gravity(3)*ctrl.rob.yaw_des];
% Cap Euler Angles
ctrl.rob.testEuler = ctrl.rob.att.euler_des(1:2,tInd);
ctrl.rob.eulerMax = 0.2;
ind = abs(ctrl.rob.testEuler)>ctrl.rob.eulerMax;
if any(ind)
   ctrl.rob.att.euler_des(ind,tInd) = ctrl.rob.eulerMax*...
      (ctrl.rob.testEuler(ind)./abs(ctrl.rob.testEuler(ind)));
end
% Attitude Control
ctrl.rob.att.eulerErr = rob.pose(4:6,tInd-1)-ctrl.rob.att.euler_des(:,tInd);
ctrl.rob.att.omega_des = -(rob.pose(4:6,tInd-1)-...
                          ctrl.rob.att.euler_des(:,tInd))/s.dt;
ctrl.rob.att.omegaErr = (rob.v(4:6,tInd-1)-ctrl.rob.att.omega_des);
ctrl.rob.tau = ctrl.rob.att.kp*ctrl.rob.att.eulerErr + ...
               ctrl.rob.att.kd*(ctrl.rob.att.omegaErr);
% % Cap Applied Torque
% ctrl.rob.tauMax = 10;
% ind = abs(ctrl.rob.tau)>ctrl.rob.tauMax;
% if any(ind)
%    ctrl.rob.tau(ind) = ctrl.rob.tauMax*ctrl.rob.tau(ind)./abs(ctrl.rob.tau(ind));
% end

% Force Control (UPenn)
ctrl.rob.F = -rob.m*ctrl.rob.a_des(3,1);
% ctrl.rob.F = rob.m*rob.gravity(3,1)/(cos(rob.pose(4,tInd))*cos(rob.pose(5,tInd)));
% Thrust During Unstable Flight
if ctrl.rob.F<0
   ctrl.rob.F = 0;
elseif ctrl.rob.F>20
   ctrl.rob.F = 20;
end


%% Update Robot Pose with Quadrotor Dynamics

% Add Simulated Robot Dynamics Noise
if est.dynamicsNoise==1
   est.m_v(1:3,tInd) = normrnd(0,est.dynCov,[3,1]);
   est.m_v(4:6,tInd) = normrnd(0,est.rotCov,[3,1]);
end

% Update Orientation
rob.a(4:6,tInd) = rob.J_inv*ctrl.rob.tau-...
                  rob.J_inv*rob.Omega*rob.J*rob.v(4:6,tInd-1);
rob.v(4:6,tInd) = rob.a(4:6,tInd)*s.dt+rob.v(4:6,tInd-1);
rob.Omega = skewSymMat(rob.v(4:6,tInd));
R_r1r2 = expm(rob.Omega*s.dt);
trans.R_wr = trans.R_wr*R_r1r2*euler2rot(est.m_v(4:6,tInd)*s.dt);
rob.pose(4:6,tInd) = rot2euler(trans.R_wr);

% Update Position
rob.a(1:3,tInd) = rob.gravity-(ctrl.rob.F/rob.m)*trans.R_wr*[0;0;1];
rob.v(1:3,tInd) = rob.a(1:3,tInd)*s.dt+rob.v(1:3,tInd-1);
rob.pose(1:3,tInd) = rob.v(1:3,tInd)*s.dt+rob.pose(1:3,tInd-1)+...
                     est.m_v(1:3,tInd)*s.dt;

