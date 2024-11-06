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

% initialize default parameters for SPOT
initSpotDefaultParam;

% initialize the GNC unit handling phase management
initSpotPhaseMgmt;

% initialize the GNC unit handling measurement processing
initSpotMeasProc;

% initialize the GNC unit handling state estimation
initSpotEstimator;

% initialize the GNC unit handling reference generation
initSpotRefGen;

% initialize the GNC unit handling control error
initSpotCtrlErr;

% initialize the GNC unit handling control
initSpotController;

% initialize the GNC unit handling status flags
initSpotFlag;

% initialize environmental parameters
initSpotEnv;

