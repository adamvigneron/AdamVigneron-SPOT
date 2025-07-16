function [Phi, F] = discrete_STM(~, dT)
% FFNAV State Transition Matrix ===========================================
% Description: This function calculates the linearized state matrix
% (Jacobian) and the discrete-time state transition matrix, given the
% current state and the time step.
%
% Inputs:
%   state_pre - The previous state vector (unused)
%   dT        - Time step of the simulation

% Outputs:
%   Phi - State Transition Matrix
%   F   - Jacobian of the Dynamic Model
%
% Created by:  Cory Fraser - APR 01, 2023
% Latest Edit: Adam Vigneron - APR 04, 2023
% Copyright(c) 2023 by Cory Fraser
% =========================================================================


%% Partial derivatives of the 3 nonlinear equations

%Derivatives of x-acceleration equation
dxddot_dx          = 0;
dxddot_dy          = 0;
dxddot_dtheta      = 0;
dxddot_dx_dot      = 0;
dxddot_dy_dot      = 0;
dxddot_dtheta_dot  = 0;

%Derivatives of y-acceleration equation
dyddot_dx          = 0;
dyddot_dy          = 0;
dyddot_dtheta      = 0;
dyddot_dx_dot      = 0;
dyddot_dy_dot      = 0;
dyddot_dtheta_dot  = 0;

%Derivatives of theta-acceleration equation
dtheta_ddot_dx          = 0;
dtheta_ddot_dy          = 0;
dtheta_ddot_dtheta      = 0;
dtheta_ddot_dx_dot      = 0;
dtheta_ddot_dy_dot      = 0;
dtheta_ddot_dtheta_dot  = 0;


%% Assembling the Jacobian (6x6 Matrix)

F11 = zeros(3,3);
F12 = eye(3,3);

F21 = [dxddot_dx dxddot_dy dxddot_dtheta];
F22 = [dxddot_dx_dot dxddot_dy_dot dxddot_dtheta_dot];

F31 = [dyddot_dx dyddot_dy dyddot_dtheta];
F32 = [dyddot_dx_dot dyddot_dy_dot dyddot_dtheta_dot];

F41 = [dtheta_ddot_dx dtheta_ddot_dy dtheta_ddot_dtheta];
F42 = [dtheta_ddot_dx_dot dtheta_ddot_dy_dot dtheta_ddot_dtheta_dot];

F = [   F11 F12
        F21 F22
        F31 F32
        F41 F42 ];

%% Calculate the approximate state transition matrix, Phi = expm(F*dT)

Phi = (F*dT) * ( ((F*dT)/2) * ( ((F*dT)/3) * ( ((F*dT)/4) ...
                 + eye(6,6) )   + eye(6,6) )   + eye(6,6) ) + eye(6,6);


end