function [output] = correction(input_priori, measVec, R)
% FFNAV Extended Kalman Filter ============================================
% Description: This function completes the state and covariance correction
% (measurement update) of the EKF algorithm. The resulting a posteriori
% data is used as the state estimate for the next time step.
%
% Inputs:
%   input_priori - Vector of a priori states and covariances
%   measVec      - Vector of measurements
%   R            - Measurement noise covariance matrix
%
% Outputs:
%   output      - Vector of a posteriori states and covariances
%
% Created by:  Cory Fraser - APR 01, 2023
% Latest Edit: Adam Vigneron - APR 08, 2024
% Copyright(c) 2024 by Cory Fraser
% =========================================================================


%% Initialize and assign data

% Propagated State Vector (Nx1)
state_priori = reshape( input_priori(1: 7), 7, 1 );

% Propagated State Error Covariance (NxN)
P_priori     = reshape( input_priori(8:56), 7, 7 );

% Measurement vector (Mx1)
Z_m = measVec;


%% Defining the measurement model

% Relative x, y, and theta plus inertial omega (MxN)
H = [ 1 0 0 0 0 0 0
      0 1 0 0 0 0 0
      0 0 1 0 0 0 0
      0 0 0 0 0 0 1 ];


%% Calculate the Kalman Gain

% Calculate theoretical residual covariances (MxM)
Pr_theo = H*P_priori*H' + R;

%Calculating the Kalman Gain (NxM)
K = P_priori * H' / Pr_theo;


%% Correct the state estimate and covariance

% Calulating the estimated measurements (Mx1)
Z_est = H*state_priori;

% Calculating the innovations (Mx1)
innovations = Z_m - Z_est;

% Calculating the corrections (Nx1)
correction  = K*innovations;

% Correct the state estimate (Nx1)
state_post  = state_priori + correction;

% Correct the state error covariance matrix - Joseph's Form (NxN Matrix)
P_post = (eye(7)-K*H)*P_priori*(eye(7)-K*H)' + K*R*K';


%% Converting data into output vector format

% Vectorized State Estimate Covariance (1 x N^2)
state_post = reshape (state_post, [], 1);
P_post     = reshape (    P_post, [], 1);

% Assembled output vector (1 x N^2+N)
output = [ state_post; P_post];


end  % function

