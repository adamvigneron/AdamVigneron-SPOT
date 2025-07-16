function [P0,Q0,R0] = initialize_EKF()
%% Initialize EKF =========================================================
% Description: This script defines and loads all data needed for the space-
% craft on-board software.
%
% Inputs:
%   None
% Outputs:
%   P0: Initial state covariance matrix
%   Q0: Initial dynamics covariance matrix
%   R0: Initial measurements covariance matrix
%
% Created by:  Cory Fraser - JUL 07, 2023
% Latest Edit: Adam Vigneron - APR 03, 2025
% Copyright(c) 2023 by Cory Fraser
% =========================================================================


%% Initial state error covariances

sig2_xRel     = ( (1/3) * 1e-4 )^2;  % PhaseSpace at 0.1 mm (3-sigma)
sig2_yRel     = ( (1/3) * 1e-4 )^2;  % PhaseSpace at 0.1 mm (3-sigma)
sig2_thetaRel = ( (1/3) * 1e-4 )^2;  % PhaseSpace at 0.1 mrad (3-sigma)

sig2_xRelDot     = ( 1e-1 )^2;  % worst-case initial value 0.1 m/s 
sig2_yRelDot     = ( 1e-1 )^2;  % worst-case initial value 0.1 rad/s
sig2_thetaRelDot = ( 1e-1 )^2;  % worst-case initial value 0.1 rad/s

% Assemble initial state error covariance matrix P0
P0 = diag([sig2_xRel    sig2_yRel    sig2_thetaRel ...
           sig2_xRelDot sig2_yRelDot sig2_thetaRelDot]);


%% Initial Q Matrix (Covariance of the process noise)

% Quaternion and angular rate noise components
q2_xRel     = ( (1/3) * 1e-2 )^2; % thrusters at 10 mN (3-sigma)
q2_yRel     = ( (1/3) * 1e-2 )^2; % thrusters at 10 mN (3-sigma)
q2_thetaRel = ( (1/3) * 1e-3 )^2; % thrusters at 1 mNm (3-sigma)

% Assemble Q matrix 
Q0 = diag([0 0 0 q2_xRel q2_yRel q2_thetaRel]);


%% Initial R Matrix (Covariance of the measurement noise)

r2_xRel     = ( (1/3) * 1e-4 )^2; % PhaseSpace at 0.1 mm (3-sigma)
r2_yRel     = ( (1/3) * 1e-4 )^2; % PhaseSpace at 0.1 mm (3-sigma)
r2_thetaRel = ( (1/3) * 1e-4 )^2; % PhaseSpace at 0.1 mrad (3-sigma)

% Assemble R matrix
R0 = diag([r2_xRel r2_yRel r2_thetaRel]);

