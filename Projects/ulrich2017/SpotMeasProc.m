function [proc] = SpotMeasProc(phase, meas, paramMeasProc)

    %% initialization of output variables
    
    coords   = enumeration('SpotCoord');
    numCoord = length(coords);

    proc = zeros(numCoord,1);
    
    
    %% loop over all coordinates

    for k = 1:numCoord
        
        coord = coords(k);

        %% select a measurement processing method
        myFun = paramMeasProc(phase,coord).fun;
        
        switch myFun
    
            case SpotGnc.procNone

                proc(coord) = meas(coord);

            otherwise
                error('SpotMeasProc.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function

