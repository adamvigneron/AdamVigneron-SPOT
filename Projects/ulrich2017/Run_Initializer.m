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

% initialize control parameters for the manipulator
initSpotArmCtrlParam;

% initialize the GNC unit handling reference generation
initSpotRefGen;

