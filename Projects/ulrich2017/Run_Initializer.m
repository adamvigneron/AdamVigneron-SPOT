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


%% Iterative Learning Control

% "Iterative Learning Control of Spacecraft Proximity Operations Based on Confidence Level"
% Steve Ulrich and Kirk Hovell
% AIAA GNC 2017
% https://arc.aiaa.org/doi/10.2514/6.2017-1046 

% This script simulates the numerical example of Section III.C

% select a value for beta
% 0 - feed-forward blocked
% 1 - feed-forward applied
% (decimal values also possible)
beta = 0.25;


%% Define feed-forward signal

try 
    load('Saved Data/SimulationData_2024_3_14_13_39_19_4889/dataPacket_SIM.mat');
    myTime = dataClass.Time_s;
    myDataX = dataClass.RED_Fx_N;
    myDataY = dataClass.RED_Fy_N;

    fprintf('Run_Initializer.m:\n');
    fprintf('  feed-forward signal loaded from datafile\n\n');

catch ME
    if (strcmp(ME.identifier,'MATLAB:load:couldNotReadFile'))
        myTime = 0:baseRate:tsim;
        myDataX = 0*myTime;
        myDataY = 0*myTime;

        fprintf('Run_Initializer.m:\n');
        fprintf('  feed-forward datafile not found; feed-forward signal set to zero\n\n')

    elseif (strcmp(ME.identifier,'MATLAB:nonExistentField'))
        myTime = 0:baseRate:tsim;
        myDataX = 0*myTime;
        myDataY = 0*myTime;

        fprintf('Run_Initializer.m:\n');
        fprintf(['  ' ME.message(1:(end-1)) ' within feed-forward datafile; feed-forward signal set to zero\n\n'])

    else
        rethrow(ME)
    end
end

RED_Fx_Fwd_N = timeseries(myDataX,myTime);
RED_Fy_Fwd_N = timeseries(myDataY,myTime);

