function [P0,Q0,R0] = initialize_EKF( time_step, relMeas )
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

sig2_omega = ( (1/3) * 1e-3 )^2;  % IMU at 1 mrad/s2 (3-sigma)

% Assemble initial state error covariance matrix P0
P0 = diag([sig2_xRel    sig2_yRel    sig2_thetaRel ...
           sig2_xRelDot sig2_yRelDot sig2_thetaRelDot ...
           sig2_omega]);


%% Initial Q Matrix (Covariance of the process noise)

% Quaternion and angular rate noise components (doubled for relative frame)
q2_xRel     = ( (1/3) * 2e-1 / 11.2970 )^2; % thrusters at 2*100 mN (3-sigma)
q2_yRel     = ( (1/3) * 2e-1 / 11.2970 )^2; % thrusters at 2*100 mN (3-sigma)
q2_thetaRel = ( (1/3) * 2e-2 /  0.1982 )^2; % thrusters at 2*10 mNm (3-sigma)

% single-axis double integrator, input matrix, zero-order hold
Gamma = [ time_step^2 / 2 ; time_step ];

% Assemble Q matrix (discrete time)
Q0 = zeros(7,7);
Q0([1 4],[1 4]) = Gamma * Gamma' * q2_xRel;
Q0([2 5],[2 5]) = Gamma * Gamma' * q2_yRel;
Q0([3 6],[3 6]) = Gamma * Gamma' * q2_thetaRel;
Q0(    7,    7) = time_step^2    * q2_thetaRel;  % this is a bit too noisy


%% Initial R Matrix (Covariance of the measurement noise)

switch relMeas

    case { SpotGnc.estEkfStereo , SpotGnc.estEkfPolarStereo }
        r2_xRel     = ( (1/3) * 5e-2 )^2; % stereo at 5 cm (3-sigma)
        r2_yRel     = ( (1/3) * 5e-2 )^2; % stereo at 5 cm (3-sigma)
        r2_thetaRel = ( (1/3) * 2e-1 )^2; % stereo at 0.2 rad (3-sigma)

    case { SpotGnc.estEkfLidar , SpotGnc.estEkfPolarLidar }
        r2_xRel     = ( (1/3) * 5e-2 )^2; % lidar at 5 cm (3-sigma)
        r2_yRel     = ( (1/3) * 5e-2 )^2; % lidar at 5 cm (3-sigma)
        r2_thetaRel = ( (1/3) * 2e-1 )^2; % lidar at 0.2 rad (3-sigma)

    otherwise
        error('initailize_EKF.m:\n  sensor not defined for relative EKF')

end

r2_omega = ( (1/3) * 1e-3 )^2;  % IMU at 1 mrad/s2 (3-sigma)

% Assemble R matrix
R0 = diag([r2_xRel r2_yRel r2_thetaRel r2_omega]);

