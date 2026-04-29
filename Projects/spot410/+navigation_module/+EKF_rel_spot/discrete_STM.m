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

xBr         = state_pre(1);
yBr         = state_pre(2);
thetaRedDot = state_pre(8);

% Derivatives of x-acceleration equation
% xBr_ddot = ux + thetaRed_dot^2*xBr + 2*thetaRed_dot*yBr_dot + thetaRed_ddot*yBr;

dxddot_dxBr         = thetaRedDot^2;
dxddot_dyBr         = 0;  % + thetaRed_ddot;
dxddot_dthetaBr     = 0;
dxddot_dthetaRed    = 0;
dxddot_dxBrDot      = 0;
dxddot_dyBrDot      = 0;  % + 2*thetaRedDot;
dxddot_dthetaBrDot  = 0;
dxddot_dthetaRedDot = 2*thetaRedDot*xBr;  % + 2*yBrDot;

% Derivatives of y-acceleration equation
% uy + thetaRed_dot^2*yBr - 2*thetaRed_dot*xBr_dot - thetaRed_ddot*xBr;

dyddot_dxBr         = 0;  % - thetaRed_ddot;
dyddot_dyBr         = thetaRedDot^2;
dyddot_dthetaBr     = 0;
dyddot_dthetaRed    = 0;
dyddot_dxBrDot      = 0;  % - 2*thetaRedDot;
dyddot_dyBrDot      = 0;
dyddot_dthetaBrDot  = 0;
dyddot_dthetaRedDot = 2*thetaRedDot*yBr;  % - 2*xBrDot;


%% Assembling the Jacobian (8x8 Matrix)

F11 = zeros(4,4);
F12 = eye(4,4);

F21 = [dxddot_dxBr    dxddot_dyBr    dxddot_dthetaBr    dxddot_dthetaRed];
F22 = [dxddot_dxBrDot dxddot_dyBrDot dxddot_dthetaBrDot dxddot_dthetaRedDot];

F31 = [dyddot_dxBr    dyddot_dyBr    dyddot_dthetaBr    dyddot_dthetaRed];
F32 = [dyddot_dxBrDot dyddot_dyBrDot dyddot_dthetaBrDot dyddot_dthetaRedDot];

F41 = zeros(2,4);
F42 = zeros(2,4);

F = [ F11 F12
      F21 F22
      F31 F32
      F41 F42 ];

%% Calculate the approximate state transition matrix, Phi = expm(F*dT)

Phi = (F*dT) * ( ((F*dT)/2) * ( ((F*dT)/3) * ( ((F*dT)/4) ...
                 + eye(8,8) )   + eye(8,8) )   + eye(8,8) ) + eye(8,8);


end