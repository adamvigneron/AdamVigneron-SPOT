function [err,err_vel] = SpotCtrlErr(phase, ref, ref_vel, est, est_vel, paramCtrlErr)

    %% initialization of output and persistent variables
    
    coords   = enumeration( SpotCoord(1) );
    numCoord = length(coords);

    err     = zeros(numCoord,1);
    err_vel = zeros(numCoord,1);

    persistent prevEst;
    persistent prevErr;
    persistent prevErrVel;

    if isempty(prevEst)
        prevEst    = zeros(numCoord,1);
        prevErr    = zeros(numCoord,1);
        prevErrVel = zeros(numCoord,1);
    end
    
    
    %% loop over all coordinates

    for k = 1:numCoord
        
        coord = coords(k);

        %% select an error calculation method
        myFun = paramCtrlErr(phase,coord).fun;
        
        switch myFun
    
            case SpotGnc.errMinus

                % check whether the input is constant
                if est(coord) == prevEst(coord)

                    % if it is, keep the output constant
                    err(coord)     = prevErr(coord);
                    err_vel(coord) = prevErrVel(coord);

                else

                    % otherwise, update both outputs 
                    err(coord)     = ref(coord)     - est(coord);
                    err_vel(coord) = ref_vel(coord) - est_vel(coord);

                    % and update the persistent variables as well
                    prevEst(coord)    = est(coord);
                    prevErr(coord)    = err(coord);
                    prevErrVel(coord) = err_vel(coord);

                end
    
            case SpotGnc.errMinusWrap
    
                % check whether the input is constant
                if est(coord) == prevEst(coord)

                    % if it is, keep the output constant
                    err(coord)     = prevErr(coord);
                    err_vel(coord) = prevErrVel(coord);

                else

                    % otherwise, update both outputs 
                    errWrap = wrapToPi( ref(coord) ) - wrapToPi( est(coord) );
            
                    if abs(errWrap) < pi
                        err(coord) = errWrap;
                    else
                        err(coord) = errWrap - sign(errWrap)*2*pi;
                    end
        
                    err_vel(coord) = ref_vel(coord) - est_vel(coord);

                    % and update the persistent variables as well
                    prevEst(coord)    = est(coord);
                    prevErr(coord)    = err(coord);
                    prevErrVel(coord) = err_vel(coord);

                end                    
        
            case SpotGnc.errArmSetpoint

                % pass the reference signal through directly
                err(coord)     = ref(coord);
                err_vel(coord) = ref_vel(coord);
        
            otherwise
                error('SpotCtrlErr.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function

