function [output] = propagation(ekf_pre, input_pre, time_step, Q)
% FFNAV Extended Kalman Filter Propagation ================================
% Description: This function completes the state and covariance propagation
% (time update) of the EKF algorithm. The resulting a priori data is
% then passed to the correction phase of the EKF algorithm.
%
% Inputs:
%   ekf_pre     - Vector of previous states and covariances
%   input_pre   - Vector of input parameters 
%   time_step   - Time step of the simulation
%   Q           - Process noise covariance matrix
%
% Outputs:
%   output      - Vector of a priori states and covariances
%
% Created by:  Cory Fraser - APR 01, 2023
% Latest Edit: Adam Vigneron - APR 03, 2023
% Copyright(c) 2023 by Cory Fraser
% ========================================================================


%% Initialize and assign data

% Previous State Vector (7x1 Matrix)
state_pre =  ekf_pre(1:7);

% Previous State Error Covariance (7x7 Matrix)
P_pre = reshape( ekf_pre(8:56), 7, 7);

% relative control
du = input_pre( [SpotCoord.xRed; ...
                 SpotCoord.yRed; ...
                 SpotCoord.thetaRed] );


%% EKF Propagation Step

% Runge-Kutta-Merson integration to propagate state vector (7x1 Matrix)
state_priori = utilities.rkm_54_integration( ...
    @navigation_module.EKF_rel_spot.spot_dot, state_pre, du, time_step);

% Discrete-time state transition matrix (7x7 Matrix)
phi = navigation_module.EKF_rel_spot.discrete_STM( ...
    state_pre, time_step);

% propagate the covariance
P_priori = phi*P_pre*phi' + Q;


%% Convert data into output vector format

state_priori = reshape( state_priori, [], 1);
P_priori     = reshape( P_priori, [], 1);
output       = [ state_priori; P_priori ];

end

