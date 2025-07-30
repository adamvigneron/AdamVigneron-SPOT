% The following script is the initializer for SPOT 4.1; in this script,
% users define all initials parameters and/or constants required for
% simulation and experiment.

clear;
clc;
close all force;

warning('off','all')


%% Start the graphical user interface

appHandle = GUI_v4_1_Main;


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

% load the BLACK-with-cone inspection data
load('+navigation_module/ExperimentData_RED_2025_7_24_14_6_22_4009_1.mat')


%% For those who want to run simulations without using the GUI:

% % Load previous GUI states using this public facing function
% appHandle.LoadDataPublicFcn(cd,'SampleGUIState.mat');
%
% % Set the diagram to run (must be set again after each load)
% appHandle.AvailableDiagramsDropDown.Value = "Template_v4_1_0_2024b_Jetson.slx";
% 
% % Ensure the diagram is loaded
% open(appHandle.AvailableDiagramsDropDown.Value);
% 
% % Edit active platforms
% appHandle.REDCheckBox.Value    = 1;
% appHandle.BLACKCheckBox.Value  = 0;
% appHandle.BLUECheckBox.Value   = 0;
% appHandle.ARMCheckBox.Value    = 0;
% 
% appHandle.ConfirmSettings();
% 
% % Edit initial conditions
% appHandle.SubAppInitialConditions.REDInitialX.Value  = 1.2;  % [m]
% appHandle.SubAppInitialConditions.REDInitialY.Value  = 1.2;  % [m]
% appHandle.SubAppInitialConditions.REDInitialTh.Value = 90;   % [deg]
% 
% appHandle.SubAppInitialConditions.UpdateInitialConditions();
% 
% % Edit mass properties
% appHandle.SubAppMassProperties.OverridePropertiesCheckBox.Value = 1;
% appHandle.SubAppMassProperties.MassRedEditField.Value = 12.035;    % [kg]
% appHandle.SubAppMassProperties.InertiaRedEditField.Value = 0.19854;% [kgm2]
% 
% appHandle.SubAppMassProperties.UpdateMassProperties();
% 
% % Edit phase durations
% appHandle.SubPhase1EditField.Value = 10;       % [s]
% appHandle.SubPhase2EditField.Value = 5;        % [s]
% appHandle.SubPhase3EditField.Value = 28;       % [s]
% appHandle.SubPhase4EditField.Value = 115;      % [s]
% appHandle.DurPhase0EditField.Value = 10;       % [s]
% appHandle.DurPhase1EditField.Value = 5;        % [s]
% appHandle.DurPhase2EditField.Value = 40;       % [s]
% appHandle.DurPhase4EditField.Value = 30;       % [s]
% appHandle.DurPhase5EditField.Value = 20;       % [s] 
% 
% appHandle.UpdateTimes();
% 
% % Execute a simulation
% appHandle.RunSimulationPublicFcn();
% 
% % Manipulate the simulation data
% figure()
% plot(dataClass.Time_s.Data, dataClass.RED_Px_m.Data,'-k')
% grid on
% hold on
% axis tight
% xlabel('Time [s]')
% ylabel('Position - X [m]')


