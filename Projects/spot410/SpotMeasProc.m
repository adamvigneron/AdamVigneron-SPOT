function [proc] = SpotMeasProc(phase, meas, paramMeasProc)

    %% initialization of output and persistent variables
    
    sensors   = enumeration( SpotSensor(1) );
    numSensor = length(sensors);

    proc = zeros(numSensor,1);

    persistent prevProc;
    persistent sensorBias;

    if isempty(prevProc)
        prevProc   = zeros(numSensor,1);
        sensorBias = zeros(numSensor,1);
    end
    
    
    %% loop over all measurements

    for k = 1:numSensor
        
        sensor = sensors(k);

        %% select a measurement processing method
        myFun = paramMeasProc(phase,sensor).fun;
        
        switch myFun
    
            case SpotGnc.procNone

                proc(sensor) = meas(sensor);


            case SpotGnc.procAngle

                measDelta = wrapToPi( meas(sensor) - prevProc(sensor) );

                proc(sensor) = prevProc(sensor) + measDelta;

                prevProc(sensor) = proc(sensor);


            case SpotGnc.procAngleQuadrant

                % wrap to pi/4
                fourIn    = 4 * ( meas(sensor) - prevProc(sensor) );
                fourOut   = wrapToPi(fourIn);
                measDelta = fourOut / 4;

                proc(sensor) = prevProc(sensor) + measDelta;

                prevProc(sensor) = proc(sensor);


            case SpotGnc.procImuBias
                
                % in Phase0, we aren't moving and can estimate the IMU bias
                if phase == SpotPhase.Phase0

                    k1 = paramMeasProc(phase,sensor).k1;  % tau
                    k2 = paramMeasProc(phase,sensor).k2;  % baseRate

                    % exponential moving average in discrete time
                    alpha = 1 - exp(-k1/k2);

                    % update the stored sensor bias
                    sensorBias(sensor) = alpha * meas(sensor) + (1-alpha) * sensorBias(sensor);

                    % output the uncorrected measurement
                    proc(sensor) = meas(sensor);
                 
                % otherwise, we correct the IMU using the estimated bias
                else
                    proc(sensor) = meas(sensor) - sensorBias(sensor);

                end
 

            case SpotGnc.procImuPhasespace

                % substitute the phasespace rate for the IMU rate
                proc(sensor) = meas( int32(sensor) - 12);


            otherwise
                error('SpotMeasProc.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotSensor(%d).\n\n', int32(myFun), int32(phase), int32(sensor))
    
        end % switch myFun

    end % loop sensors

end % function

