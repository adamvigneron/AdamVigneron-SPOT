function [sensor_mode] = select_sensor_mode(sensor_data)
% Pre-process Measurements ================================================
% Description: This function processes the sensor data provided to the
% navigation module, and determines which measurements and corresponding
% EKF to use within the navigation routine.
%
% Inputs:
%   sensor_data: struct containing data vectors for the measurement sensors
%
% Outputs:
%   sensor_mode: string specifying the current sensor mode
%
% Created by:  Cory Fraser - MAY 22, 2023
% Latest Edit: Adam Vigneron - APR 02, 2025
% Copyright(c) 2023 by Cory Fraser
% =========================================================================
%% Logic to determine which EKF mode to use based on sensor availability 

% EKF Modes for SPOT x-y-theta estimation
% 'EKF_PhaseSpace' = EKF with absolute PhaseSpace measurements for relative state estimation

% Flag for absolute servicer measurements
flag_servicer_PhaseSpace_available = ...
    ~anymissing( sensor_data( [SpotSensor.xRedPhasespace, ...
                               SpotSensor.yRedPhasespace, ...
                               SpotSensor.thetaRedPhasespace ]));

% Flag for absolute client measurements
flag_client_PhaseSpace_available = ...
    ~anymissing( sensor_data( [SpotSensor.xBlackPhasespace, ...
                               SpotSensor.yBlackPhasespace, ...
                               SpotSensor.thetaBlackPhasespace ]));

% Mode Selection Logic for SPOT Estimates
if flag_servicer_PhaseSpace_available && flag_client_PhaseSpace_available
    sensor_mode = 'EKF_PhaseSpace';
else
    sensor_mode = '';
end

end