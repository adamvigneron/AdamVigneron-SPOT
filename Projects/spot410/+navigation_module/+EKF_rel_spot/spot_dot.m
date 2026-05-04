function [f] = spot_dot(~, state, input)
% FFNAV EKF Dot ===========================================================
% Description: This function defines the nonlinear equations of relative
% motion for SPOT platforms. When passed a state vector, this
% function calculates the respective accelerations for the x, y, and
% theta functions, and returns them to the calling function in a vector.
%
% Inputs:
%   ~ - used for input time, only needed for use with ODE solver
%   state - The current state vector
%   input - Control inputs (differential)
%
% Outputs:
%   f       - Differential equation output
%
% Created by:  Cory Fraser - APR 01, 2023
% Latest Edit: Adam Vigneron - APR 04, 2025
% Copyright(c) 2023 by Cory Fraser
% =========================================================================


%% External (differential) control forces (i.e., accelerations)

if exist('input', 'var')
    ux     = input(1);
    uy     = input(2);
else
    ux     = 0;
    uy     = 0;
end

xBr         = state(1);
yBr         = state(2);

%% Derivatives of the state variables

% our positions and rotations map to their rates
xBr_dot      = state(5);
yBr_dot      = state(6);
thetaBr_dot  = state(7);
thetaRed_dot = state(8);

% acceleration in a rotating reference frame is given as
%   aR = aI - omega x (omega x rR) - 2 omega * vR - d(omega)/dt x rR

% for now, we're going to ignore the second two terms
% as (xBr_dot, yBr_dot, thetaRed_ddot) are all zero-mean
xBr_ddot = ux + thetaRed_dot^2*xBr;  % + 2*thetaRed_dot*yBr_dot + thetaRed_ddot*yBr;
yBr_ddot = uy + thetaRed_dot^2*yBr;  % - 2*thetaRed_dot*xBr_dot - thetaRed_ddot*xBr;

% relative theta acceleration is a function of thetaBlack_ddot and thetaRed_ddot
%   thetaBlack_ddot - we don't know anything about this
%   thetaRed_ddot   - this is a function of utheta which is noisy and zero-mean
% since we have great IMU measurements of thetaRedDot, we choose not to propagate
thetaBr_ddot  = 0;  % -1 * utheta
thetaRed_ddot = 0;  % utheta

%% Assembling the derivative of the state vector

f = [ xBr_dot;  yBr_dot;  thetaBr_dot;  thetaRed_dot; ...
      xBr_ddot; yBr_ddot; thetaBr_ddot; thetaRed_ddot ];

end

