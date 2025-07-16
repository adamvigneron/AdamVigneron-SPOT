function [proc] = SpotMeasProc(phase, meas, paramMeasProc)

    %% initialization of output and persistent variables
    
    sensors   = enumeration( SpotSensor(1) );
    numSensor = length(sensors);

    proc = zeros(numSensor,1);

    persistent prevProc;

    if isempty(prevProc)
        prevProc = zeros(numSensor,1);
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

            otherwise
                error('SpotMeasProc.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotSensor(%d).\n\n', int32(myFun), int32(phase), int32(sensor))
    
        end % switch myFun

    end % loop sensors

end % function

