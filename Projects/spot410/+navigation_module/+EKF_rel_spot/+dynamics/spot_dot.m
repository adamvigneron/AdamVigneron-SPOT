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


%% Derivatives of the state variables

x_dot      = state(4);
y_dot      = state(5);
theta_dot  = state(6);
x_ddot     = ux;
y_ddot     = uy;
theta_ddot = utheta;


%% Assembling the derivative of the state vector

f = [ x_dot; y_dot; theta_dot; x_ddot; y_ddot; theta_ddot ];


end

