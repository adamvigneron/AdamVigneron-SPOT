function [state_output] = rkm_54_integration(dyn_func, state_pre, du, T)
% Runge-Kutta-Merson Integration ==========================================
% Description: This function completes a numerical intergration of a
% specified dynamics equation, using the 5-stage, 4th-order Runge-Kutta
% formulation. 
%
% Inputs:
%   dyn_func  - Function handle pointing to the dynamics equations
%   state_pre - Vector of states at the previous time step
%   du        - Vector of dynamics inputs
%   T         - Integration time step
%
% Outputs:
%   state_output - Vector containing the propagated states
%
% Refs:
% [1] https://encyclopediaofmath.org/wiki/Kutta-Merson_method
% [2] J. De Lafontaine, J. Buijs, P. Vuilleumier, P. Van den Braembussche, and
%       K. Mellab, Development of the PROBA Attitude Control and Navigation
%       Software", in Spacecraft Guidance, Navigation and Control Systems, 
%       vol. 425, 2000, pp. 427-441.
%
% Created by:  Cory Fraser - SEP 03, 2023
% Latest Edit: Cory Fraser - SEP 03, 2023
% Copyright(c) 2023 by Cory Fraser
% ========================================================================
%% Runge-Kutta-Merson integration to propagate state vector (Nx1 Matrix)

% Call the dynamics function to evaluate intermediate products

%du0
%du_1_3
%du_1_2
%du1

k1 = dyn_func([], state_pre, du);
k2 = dyn_func([], state_pre + (1/3)*k1*T, du);
k3 = dyn_func([], state_pre + (1/6)*k1*T + (1/6)*k2*T, du);
k4 = dyn_func([], state_pre + (1/8)*k1*T + (3/8)*k3*T, du);
k5 = dyn_func([], state_pre + (1/2)*k1*T - (3/2)*k3*T + 2*k4*T, du);


% Calculate the propagated state vector
state_output = state_pre + (1/6)*(k1 + 4*k4 + k5)*T;

end