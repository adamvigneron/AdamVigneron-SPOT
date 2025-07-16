function EKF_output = query_ekf(sensor_mode, EKF_output_pre, ...
    this_sensor_data, input_pre, EKF_dt, Q0, R0)
% EKF Query ===============================================================
% Description: This function is used whenever an estimated state is
% requested by the navigation suite. The function will execute the standard
% EKF algorithm, first propagating the dynamics to update the state
% estimate to the current time, and then correcting the state estimate
% using measurements if the measurements are available. If no measurements
% are available at the current time, the EKF only performs the propagation
% portion of the state estimate.
%
% Inputs:
% sensor_mode, EKF_output_pre, this_sensor_data, input_pre, EKF_dt, Q0, R0
%
% Outputs:
%   EKF_output: vectorized output of the state estimate and estimation
%               error covariances
%
% Created by:  Cory Fraser - SEP 02, 2023
% Latest Edit: Adam Vigneron - APR 03, 2023
% Copyright(c) 2023 by Cory Fraser
% =========================================================================


%% Propagation Step

% NOTE: control inputs for the dynamics are contained within sensor_data

% Propagate state estimates from previous time step to a-priori estimates
EKF_output = navigation_module.EKF_rel_spot.EKF_PhaseSpace.propagation(EKF_output_pre, input_pre, EKF_dt, Q0);

%% Measurement Phase

% Logical test to see if required measurements are available
measurements_available = false;
switch sensor_mode
    case 'EKF_PhaseSpace'
        measurements_available = ~anymissing( this_sensor_data( [SpotSensor.xRedPhasespace, ...
                                                                 SpotSensor.yRedPhasespace, ...
                                                                 SpotSensor.thetaRedPhasespace, ...
                                                                 SpotSensor.xBlackPhasespace, ...
                                                                 SpotSensor.yBlackPhasespace, ...
                                                                 SpotSensor.thetaBlackPhasespace ]));
end

% If measurements are available, correct to a-posteriori estimates
if measurements_available
    EKF_output = navigation_module.EKF_rel_spot.EKF_PhaseSpace.correction(EKF_output, this_sensor_data, R0);
end

end