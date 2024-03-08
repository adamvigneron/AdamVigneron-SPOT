% The following script is the initializer for SPOT 4.0; in this script,
% users define all initials parameters and/or constants required for
% simulation and experiment.

clear;
clc;
close all force;

warning('off','all')

%% Start the graphical user interface:

run('GUI_v4_0_Main');

%% Place any custom variables or overwriting variables in this section

% As an example, here are the control parameters for the manipulator.

% Set torque limits on joints

Tz_lim_sharm                   = .1; % Shoulder Joint [Nm]

Tz_lim_elarm                   = .1; % Elbow Joint [Nm]

Tz_lim_wrarm                   = .1; % Wrist Joint [Nm]

% Transpose Jacobian controller gains:

Kp = [0.08 0 0
      0    0.08 0
      0    0    0.002];
Kv = [0.05 0 0
      0    0.05 0
      0    0    0.005];

% Initialize the PID gains for the ARM:

Kp_sharm                       = 1.5;
Kd_sharm                       = 1.0;

Kp_elarm                       = 1.2;
Kd_elarm                       = 0.8;

Kp_wrarm                       = 2;
Kd_wrarm                       = 0.6;


% Define the model properties for the joint friction:
% Based on https://ieeexplore.ieee.org/document/1511048

%Shoulder
Gamma1_sh = 0.005; 
Gamma2_sh = 5;
Gamma3_sh = 40;
Gamma4_sh = 0.015; 
Gamma5_sh = 800; 
Gamma6_sh = 0.005;

%Elbow
Gamma1_el = 0.12; 
Gamma2_el = 5;
Gamma3_el = 10;
Gamma4_el = 0.039; 
Gamma5_el = 800;
Gamma6_el = 0.000001;

%Wrist
Gamma1_wr = 0.025;
Gamma2_wr = 5;
Gamma3_wr = 40;
Gamma4_wr = 0.029;
Gamma5_wr = 800; 
Gamma6_wr = 0.02;

% Set the PWM frequency
PWMFreq = 5; %[Hz]

%% Iterative Learning Control

% "Iterative Learning Control of Spacecraft Proximity Operations Based on Confidence Level"
% Steve Ulrich and Kirk Hovell
% AIAA GNC 2017
% https://arc.aiaa.org/doi/10.2514/6.2017-1046 

% This script simulates the numerical example of Section III.C

% select a value for beta
% 0 - feed-forward blocked
% 1 - feed-forward applied
% (fractional values also possible)
beta = 0.5;


%% Define feed-forward signal

try 
    load('Saved Data/SimulationData_2024_3_7_16_51_16_809/dataPacket_SIM.mat');
    myTime = dataClass.Time_s;
    myData = dataClass.CustomUserData51;

    fprintf('Run_Initializer.m:\n');
    fprintf('  feed-forward signal loaded from datafile\n\n');

catch ME
    if (strcmp(ME.identifier,'MATLAB:load:couldNotReadFile'))
        myTime = 0:baseRate:tsim;
        myData = 0*myTime;

        fprintf('Run_Initializer.m:\n');
        fprintf('  feed-forward signal set to zero\n\n')

    else
        rethrow(ME)
    end
end

RED_Fx_Fwd_N = timeseries(myData,myTime);


%% Initialize the reference signal

% codegen - predeclare parameter struct
nPhase    = 7;
nSubphase = 4;
nCoord    = 12;

structRefGen.fun = 0;
structRefGen.k1  = 0;
structRefGen.k2  = 0;
structRefGen.k3  = 0;
structRefGen.k4  = 0;

paramRefGen = repmat(structRefGen,nPhase,nSubphase,nCoord);

% specify the phase, subphase, and coordinate of interest
phase      = 3;  % will be combined with subphase in an enumeration
subphase   = 4;
coordinate = 1;  % xRed; will be converted to an enumeration

% fun = k1 * cos( k2 * t + k3 ) + k4
paramRefGen(phase,subphase,coordinate).fun = 1;  % cosine; will be converted to an enumeration
paramRefGen(phase,subphase,coordinate).k1  = 0.85;
paramRefGen(phase,subphase,coordinate).k2  = 0.03490659;
paramRefGen(phase,subphase,coordinate).k3  = 0;
paramRefGen(phase,subphase,coordinate).k4  = xLength / 2;

% we update the definition of k4 to ensure a bumpless transfer
paramRefGen(phase,subphase,coordinate).k4  = init_states_RED(1) - paramRefGen(phase,subphase,coordinate).k1;

