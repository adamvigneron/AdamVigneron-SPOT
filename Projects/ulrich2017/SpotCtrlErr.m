function err = SpotCtrlErr(phase, est, ref, paramCtrlErr)

    %% initialization of output variables
    
    coords   = enumeration('SpotCoord');
    numCoord = length(coords);

    err = zeros(numCoord,1);
    
    
    %% loop over all coordinates

    for k = 1:numCoord
        
        coord = coords(k);

        %% select an error calculation method
        myFun = paramCtrlErr(phase,coord).fun;
        
        switch myFun
    
            case SpotGnc.errMinus
    
                err(coord) = ref(coord) - est(coord);
    
            case SpotGnc.errMinusWrap
    
                errWrap = wrapToPi( ref(coord) ) - wrapToPi( est(coord) );
    
                if abs(errWrap) < pi
                    err(coord) = errWrap;
                else
                    err(coord) = errWrap - sign(errWrap)*2*pi;
                end
    
    
            case SpotGnc.errHough

                r = ref(coord);
                e = est(coord);
        
                xRef = [cos(r) sin(r); -sin(r) cos(r)]*[1;0];
                yRef = [cos(r) sin(r); -sin(r) cos(r)]*[0;1];
        
                xAct = [cos(e) sin(e); -sin(e) cos(e)]*[1;0];
        
                errX = xRef - xAct;
                err(coord)  = -sign(yRef'*errX) * norm(errX);
        
            otherwise
                error('SpotCtrlErr.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function

