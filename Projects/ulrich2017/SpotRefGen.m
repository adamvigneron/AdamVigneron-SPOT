function ref = SpotRefGen(t, phaseSubCoord, paramRefGen)

    phase      = phaseSubCoord(1);
    subphase   = phaseSubCoord(2);
    coordinate = phaseSubCoord(3);

    myFun = paramRefGen(phase,subphase,coordinate).fun;
 
    switch myFun
        case 1  % cosine; will be replaced with an enumeration
            k1 = paramRefGen(phase,subphase,coordinate).k1;
            k2 = paramRefGen(phase,subphase,coordinate).k2;
            k3 = paramRefGen(phase,subphase,coordinate).k3;
            k4 = paramRefGen(phase,subphase,coordinate).k4;

            ref = k1 * cos( k2 * t + k3) + k4;

        otherwise
            error('SpotRefGen.m:\n  specified reference function "%s" not defined (phase %d subphase %d coordinate %d).', myFun, phase, subphase, coordinate)

    end % switch myFun

end % function

