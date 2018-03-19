%--------------------------------------------------------------------------
%
% File Name:      auroCameraModel.m
% Date Created:   2016/09/02
% Date Modified:  2016/09/05
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Move Camera in 3D Environment    
%
%--------------------------------------------------------------------------

% Add Simulated Robot Dynamics Noise
if est.dynamicsNoise==1
%    est.m_v = normrnd(0,est.q,[3,2]).*1;
   est.m_v(:,tInd) = normrnd(0,est.dynCov,[6,1]);
   est.m_v(4:6,tInd) = [0;0;0]; % no rotational noise for now
%    est.m_v(4:6,tInd) = est.m_v(4:6,tInd)./10;
%    est.m_v
end

% Camera Update in World Coordinates
ctrl.omega = [0;0;0];
trans.R_wc_new = trans.R_wc*euler2rot((ctrl.omega).*s.dt+est.m_v(4:6,tInd));
cam.pose(4:6,tInd) = rot2euler(trans.R_wc_new);
trans.poseAdd = trans.R_wc_new*ctrl.nu(:,cam.tInd)*s.dt+est.m_v(1:3,tInd)*s.dt;
cam.pose(1:3,tInd) = cam.pose(1:3,tInd-1)+trans.poseAdd;
trans.R_wc = euler2rot(cam.pose(4:6,tInd));
post.trans.R_wc(:,:,tInd) = trans.R_wc;
trans.R_cw = inv(trans.R_wc);

% Robot Update in World Coordinates
trans.R_wr = trans.R_wc*trans.R_cr;
rob.pose(1:3,tInd) = cam.pose(1:3,tInd)-trans.R_wr*(cam.offset(1:3,1));
rob.pose(4:6,tInd) = rot2euler(trans.R_wr);

% Update Camera's Tranformation For Virtual Image Acquisition
trans.camTrans = [trans.R_cw,-trans.R_cw*cam.pose(1:3,tInd); 0,0,0,1];