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
    utheta = input(3);
else
    ux     = 0;
    uy     = 0;
    utheta = 0;
end

x     = state(1);
y     = state(2);

omega = state(7);
% alpha = utheta;  % this is a bit too noisy

%% Derivatives of the state variables

x_dot      = state(4);
y_dot      = state(5);
theta_dot  = state(6);

x_ddot     = ux + 2*omega*y_dot + omega^2*x;  % + alpha*y;
y_ddot     = uy - 2*omega*x_dot + omega^2*y;  % - alpha*x;
theta_ddot = utheta;

% omega_dot  = utheta;  % this is a bit too noisy
omega_dot = 0;


%% Assembling the derivative of the state vector

f = [ x_dot; y_dot; theta_dot; x_ddot; y_ddot; theta_ddot; omega_dot ];


end

