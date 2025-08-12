function [Phi, F] = discrete_STM(state_pre, dT)
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


%% Partial derivatives of the 2 nonlinear equations

x     = state_pre(1);
y     = state_pre(2);
omega = state_pre(7);

%Derivatives of x-acceleration equation
dxddot_dx          = omega^2;
dxddot_dy          = 0;  % + omega_dot;
dxddot_dtheta      = 0;
dxddot_dx_dot      = 0;
dxddot_dy_dot      = 0;  % + 2*omega;
dxddot_dtheta_dot  = 0;
dxddot_domega      = 2*omega*x;  % + 2*y_dot

%Derivatives of y-acceleration equation
dyddot_dx          = 0;  % - omega_dot;
dyddot_dy          = omega^2;
dyddot_dtheta      = 0;
dyddot_dx_dot      = 0;  % - 2*omega;
dyddot_dy_dot      = 0;
dyddot_dtheta_dot  = 0;
dyddot_domega      = 2*omega*y;  % - 2*x_dot 


%% Assembling the Jacobian (7x7 Matrix)

F11 = zeros(3,3);
F12 = eye(3,3);
F13 = zeros(3,1);

F21 = [dxddot_dx dxddot_dy dxddot_dtheta];
F22 = [dxddot_dx_dot dxddot_dy_dot dxddot_dtheta_dot];
F23 = dxddot_domega;

F31 = [dyddot_dx dyddot_dy dyddot_dtheta];
F32 = [dyddot_dx_dot dyddot_dy_dot dyddot_dtheta_dot];
F33 = dyddot_domega;

F41 = [0 0 0];
F42 = [0 0 0];
F43 = 0;

F51 = [0 0 0];
F52 = [0 0 0];
F53 = 0;

F = [ F11 F12 F13
      F21 F22 F23
      F31 F32 F33
      F41 F42 F43
      F51 F52 F53 ];

%% Calculate the approximate state transition matrix, Phi = expm(F*dT)

Phi = (F*dT) * ( ((F*dT)/2) * ( ((F*dT)/3) * ( ((F*dT)/4) ...
                 + eye(7,7) )   + eye(7,7) )   + eye(7,7) ) + eye(7,7);


end