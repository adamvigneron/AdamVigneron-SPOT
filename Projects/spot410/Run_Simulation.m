% run the SPOT intialization script
Run_Initializer;

% Set the diagram to run and ensure the diagram is loaded
appHandle.AvailableDiagramsDropDown.Value = "spot410.slx";
open(appHandle.AvailableDiagramsDropDown.Value);

% Edit active platforms
appHandle.REDCheckBox.Value    = 1;
appHandle.BLACKCheckBox.Value  = 1;
appHandle.BLUECheckBox.Value   = 0;
appHandle.ARMCheckBox.Value    = 0;
appHandle.ConfirmSettings();

% Execute a simulation
appHandle.RunSimulationPublicFcn();

