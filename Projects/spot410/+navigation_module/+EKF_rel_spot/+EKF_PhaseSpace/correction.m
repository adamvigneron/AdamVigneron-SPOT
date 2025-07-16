function [output] = correction(input_priori, sensors, R)
% FFNAV Extended Kalman Filter ============================================
% Description: This function completes the state and covariance correction
% (measurement update) of the EKF algorithm. The resulting a posteriori
% data is used as the state estimate for the next time step.
%
% Inputs:
%   input_priori - Vector of a priori states and covariances
%   sensors      - Vector of measurements
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
x_priori            = input_priori(1);
y_priori            = input_priori(2);
theta_priori        = input_priori(3);
x_dot_priori        = input_priori(4);
y_dot_priori        = input_priori(5);
theta_dot_priori    = input_priori(6);

state_priori =  [ x_priori; y_priori; theta_priori; ...
                  x_dot_priori; y_dot_priori; theta_dot_priori ];

% Propagated State Error Covariance (NxN)
P_priori = reshape( input_priori(7:42), 6, 6 );

% Measurement vector (Mx1)
x_m     = sensors(SpotSensor.xBlackPhasespace) ...
          - sensors(SpotSensor.xRedPhasespace);
y_m     = sensors(SpotSensor.yBlackPhasespace) ...
          - sensors(SpotSensor.yRedPhasespace);
theta_m = sensors(SpotSensor.thetaBlackPhasespace) ...
          - sensors(SpotSensor.thetaRedPhasespace);

Z_m = [x_m
       y_m
       theta_m ];


%% Defining the measurement model

% Relative x, y, and theta (MxN)
H = [ 1 0 0 0 0 0
      0 1 0 0 0 0
      0 0 1 0 0 0 ];


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

% Calculating the coorections (Nx1)
correction  = K*innovations;

% Correct the state estimate (Nx1)
state_post  = state_priori + correction;

%Correct the state error covariance matrix - Joseph's Form (NxN Matrix)
P_post = (eye(6)-K*H)*P_priori*(eye(6)-K*H)' + K*R*K';


%% Converting data into output vector format

% Vectorized State Estimate Covariance (1 x N^2)
state_post = reshape (state_post, [], 1);
P_post = reshape ( P_post, [], 1 );

% Assembled output vector (1 x N^2+N)
output = [ state_post; P_post];


end  % function

