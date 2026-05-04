function [P0,Q0,R0] = initialize_EKF( time_step )
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

sig2_xBr     = ( (1/3) * 1e-4 )^2;  % PhaseSpace at 0.1 mm (3-sigma)
sig2_yBr     = ( (1/3) * 1e-4 )^2;  % PhaseSpace at 0.1 mm (3-sigma)
sig2_thetaBr = ( (1/3) * 1e-4 )^2;  % PhaseSpace at 0.1 mrad (3-sigma)

sig2_thetaRed = ( (1/3) * 1e-4 )^2;  % PhaseSpace at 0.1 mrad (3-sigma)

sig2_xBrDot     = ( 1e-1 )^2;  % worst-case initial value 0.1 m/s 
sig2_yBrDot     = ( 1e-1 )^2;  % worst-case initial value 0.1 m/s
sig2_thetaBrDot = ( 1e-1 )^2;  % worst-case initial value 0.1 rad/s

sig2_thetaRedDot = ( (1/3) * 1e-3 )^2;  % IMU at 1 mrad/s2 (3-sigma)

% Assemble initial state error covariance matrix P0
P0 = diag([sig2_xBr    sig2_yBr    sig2_thetaBr    sig2_thetaRed ...
           sig2_xBrDot sig2_yBrDot sig2_thetaBrDot sig2_thetaRedDot]);


%% Initial Q Matrix (Covariance of the process noise)

% Quaternion and angular rate noise components (doubled for relative frame)
q2_xBr      = ( (1/3) * 2e-1 / 11.2970 )^2; % thrusters at 2*100 mN (3-sigma)
q2_yBr      = ( (1/3) * 2e-1 / 11.2970 )^2; % thrusters at 2*100 mN (3-sigma)
q2_thetaBr  = ( (1/3) * 2e-2 /  0.1982 )^2; % thrusters at 2*10 mNm (3-sigma)
q2_thetaRed = ( (1/3) * 1e-2 /  0.1982 )^2; % thrusters at 1*10 mNm (3-sigma)

% single-axis double integrator, input matrix, zero-order hold
Gamma = [ time_step^2 / 2 ; time_step ];

% Assemble Q matrix (discrete time)
Q0 = zeros(8,8);
Q0([1 5],[1 5]) = Gamma * Gamma' * q2_xBr;
Q0([2 6],[2 6]) = Gamma * Gamma' * q2_yBr;
Q0([3 7],[3 7]) = Gamma * Gamma' * q2_thetaBr;
Q0([4 8],[4 8]) = Gamma * Gamma' * q2_thetaRed;


%% Initial R Matrix (Covariance of the measurement noise)

r2_xBr     = ( (1/3) * 5e-3 )^2; % lidar at 5 mm (3-sigma)
r2_yBr     = ( (1/3) * 5e-3 )^2; % lidar at 5 mm (3-sigma)
r2_thetaBr = ( (1/3) * 1e-2 )^2; % lidar at 10 mrad (3-sigma)

r2_thetaRed = ( (1/3) * 1e-4 )^2; % PhaseSpace at 0.1 mrad (3-sigma)

r2_thetaRedRate = ( (1/3) * 1e-3 )^2;  % IMU at 1 mrad/s2 (3-sigma)

% Assemble R matrix
R0 = diag([r2_xBr r2_yBr r2_thetaBr r2_thetaRed r2_thetaRedRate]);

