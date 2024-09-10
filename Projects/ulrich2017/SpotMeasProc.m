function [proc] = SpotMeasProc(phase, meas, paramMeasProc)

    %% initialization of output variables
    
    coords   = enumeration('SpotCoord');
    numCoord = length(coords);

    proc = zeros(numCoord,1);

    % persistent variables - definition
    persistent REPEAT_COUNT;
    persistent LAST_PRED;
    persistent LAST_MEAS;
    persistent LAST_VEL;

    % persistent variables - initialization
    if isempty(REPEAT_COUNT)
        REPEAT_COUNT = zeros(numCoord,1);
        LAST_PRED    = zeros(numCoord,1);
        LAST_MEAS    = zeros(numCoord,1);
        LAST_VEL     = zeros(numCoord,1);
    end
    
    
    %% loop over all coordinates

    for k = 1:numCoord
        
        coord = coords(k);

        %% select an error calculation method
        myFun = paramMeasProc(phase,coord).fun;
        
        switch myFun
    
            case SpotGnc.procNone
    
                proc(coord) = meas(coord);

            case SpotGnc.procFreeze

                DELTA_T_POS   = paramMeasProc(phase,coord).k1;
                DELTA_T_VEL   = paramMeasProc(phase,coord).k2;
                VEL_THRESHOLD = paramMeasProc(phase,coord).k3;
                MAX_REPEATS   = paramMeasProc(phase,coord).k4;

                if meas(coord) == LAST_MEAS(coord)

                    REPEAT_COUNT(coord) = REPEAT_COUNT(coord) + 1;

                    thisPred = LAST_PRED(coord) + DELTA_T_POS * LAST_VEL(coord);

                    if REPEAT_COUNT(coord) > MAX_REPEATS
                        proc(coord) = thisPred;
                    else
                        proc(coord) = meas(coord);
                    end

                    LAST_PRED(coord) = thisPred;
                    % LAST_MEAS unchanged
                    % LAST_VEL unchanged

                else

                    testPred = ( meas(coord) - LAST_PRED(coord) ) / DELTA_T_VEL;
                    testMeas = ( meas(coord) - LAST_MEAS(coord) ) / DELTA_T_VEL;

                    flagPred = ( abs(testPred) > (VEL_THRESHOLD * abs(LAST_VEL(coord))) );
                    flagMeas = ( abs(testMeas) > (VEL_THRESHOLD * abs(LAST_VEL(coord))) );

                    if REPEAT_COUNT(coord) && flagPred && ~flagMeas
                        thisPred = LAST_PRED(coord) + DELTA_T_POS * LAST_VEL(coord);

                        proc(coord) = thisPred;

                        LAST_PRED(coord) = thisPred;
                        LAST_MEAS(coord) = meas(coord);
                        % LAST_VEL unchanged;

                    else
                        REPEAT_COUNT(coord) = 0;

                        proc(coord) = meas(coord);

                        thisVel = (meas(coord) - LAST_MEAS(coord)) / DELTA_T_VEL;

                        LAST_MEAS(coord) = meas(coord);
                        LAST_PRED(coord) = meas(coord);
                        LAST_VEL(coord)  = thisVel;
                    end

                end

            otherwise
                error('SpotMeasProc.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function

