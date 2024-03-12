function ref = SpotRefGen(t, phase, coord, paramRefGen)

    myFun = paramRefGen(phase,coord).fun;
 
    switch myFun

        case SpotRef.constant
            k1 = paramRefGen(phase,coord).k1;

            ref = k1;

        case SpotRef.cosine
            k1 = paramRefGen(phase,coord).k1;
            k2 = paramRefGen(phase,coord).k2;
            k3 = paramRefGen(phase,coord).k3;
            k4 = paramRefGen(phase,coord).k4;

            ref = k1 * cos( k2 * t + k3) + k4;

        otherwise

            error('SpotRefGen.m:\n  function SpotRef(%d).fun not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))

    end % switch myFun

end % function

