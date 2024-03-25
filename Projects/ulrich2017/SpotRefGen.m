function ref = SpotRefGen(t, phase, coord, paramRefGen)

    myFun = paramRefGen(phase,coord).fun;
 
    switch myFun

        case SpotKey.refConstant
            k1 = paramRefGen(phase,coord).k1;  % constant reference

            ref = k1;

        case SpotKey.refCosine
            k1 = paramRefGen(phase,coord).k1;  % amplitude
            k2 = paramRefGen(phase,coord).k2;  % frequency
            k3 = paramRefGen(phase,coord).k3;  % phase
            k4 = paramRefGen(phase,coord).k4;  % offset

            ref = k1 * cos( k2 * t + k3 ) + k4;

        case SpotKey.refSine
            k1 = paramRefGen(phase,coord).k1;  % amplitude
            k2 = paramRefGen(phase,coord).k2;  % frequency
            k3 = paramRefGen(phase,coord).k3;  % phase
            k4 = paramRefGen(phase,coord).k4;  % offset

            ref = k1 * sin( k2 * t + k3 ) + k4;

        case SpotKey.refCosineSpinup
            k1 = paramRefGen(phase,coord).k1;  % amplitude
            k2 = paramRefGen(phase,coord).k2;  % frequency
            k3 = paramRefGen(phase,coord).k3;  % phase
            k4 = paramRefGen(phase,coord).k4;  % offset
            k5 = paramRefGen(phase,coord).k5;  % spin-up time

            % define a few convenience variables
            omgSpin = k2;
            tSpin   = k5;

            % calculate total angle elapsed
            if t < tSpin
                % we're in spin-up
                alpha = 0.5 * omgSpin/tSpin * t^2;
            else
                % we're already spun-up
                alpha = 0.5 * omgSpin*tSpin + omgSpin * (t-tSpin);
            end

            % total angle elapsed replaces the omega*t term
            ref = k1 * cos( alpha + k3 ) + k4;

        case SpotKey.refSineSpinup
            k1 = paramRefGen(phase,coord).k1;  % amplitude
            k2 = paramRefGen(phase,coord).k2;  % frequency
            k3 = paramRefGen(phase,coord).k3;  % phase
            k4 = paramRefGen(phase,coord).k4;  % offset
            k5 = paramRefGen(phase,coord).k5;  % spin-up time

            % define a few convenience variables
            omgSpin = k2;
            tSpin   = k5;

            % calculate total angle elapsed
            if t < tSpin
                % we're in spin-up
                alpha = 0.5 * omgSpin/tSpin * t^2;
            else
                % we're already spun-up
                alpha = 0.5 * omgSpin*tSpin + omgSpin * (t-tSpin);
            end

            % total angle elapsed replaces the omega*t term
            ref = k1 * sin( alpha + k3 ) + k4;

        case SpotKey.refPolyWrap
            k1 = paramRefGen(phase,coord).k1;  % initial angle
            k2 = paramRefGen(phase,coord).k2;  % initial rate

            ref = wrapToPi( k1 + k2 * t );

        otherwise
            error('SpotRefGen.m:\n  function SpotKey(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))

    end % switch myFun

end % function

