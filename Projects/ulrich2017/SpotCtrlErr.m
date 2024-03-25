function err = SpotCtrlErr(ref, act, phase, coord, paramCtrlErr)

    myFun = paramCtrlErr(phase,coord).fun;
    
    switch myFun

        case SpotKey.errMinus

            err = ref - act;

        case SpotKey.errMinusWrap

            errWrap = wrapToPi(ref) - wrapToPi(act);

            if abs(errWrap) < pi
                err = errWrap;
            else
                err = errWrap - sign(errWrap)*2*pi;
            end


        case SpotKey.errHough
    
            xRef = [cos(ref) sin(ref); -sin(ref) cos(ref)]*[1;0];
            yRef = [cos(ref) sin(ref); -sin(ref) cos(ref)]*[0;1];
    
            xAct = [cos(act) sin(act); -sin(act) cos(act)]*[1;0];
    
            errX = xRef - xAct;
            err  = -sign(yRef'*errX) * norm(errX);
    
        otherwise
            error('SpotCtrlErr.m:\n  function SpotKey(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))

    end % switch myFun

end % function

