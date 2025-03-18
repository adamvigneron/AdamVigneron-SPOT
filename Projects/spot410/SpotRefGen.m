function [ref,ref_vel] = SpotRefGen(phase, t, paramRefGen)

    %% initialization of output variables

    coords   = enumeration('SpotCoord');
    numCoord = length(coords);

    ref     = zeros(numCoord,1);
    ref_vel = zeros(numCoord,1);

    
    %% loop over all coordinates

    for k = 1:numCoord
        
        coord = coords(k);

        %% select a reference generation method
        myFun = paramRefGen(phase,coord).fun;
     
        switch myFun
    
            case SpotGnc.refConstant
                k1 = paramRefGen(phase,coord).k1;  % constant reference
    
                ref(coord)     = k1;
                ref_vel(coord) = 0;
    
            case SpotGnc.refCosine
                k1 = paramRefGen(phase,coord).k1;  % amplitude
                k2 = paramRefGen(phase,coord).k2;  % frequency
                k3 = paramRefGen(phase,coord).k3;  % phase
                k4 = paramRefGen(phase,coord).k4;  % offset
    
                ref(coord)     = k1 * cos( k2 * t + k3 ) + k4;
                ref_vel(coord) = k1 * sin( k2 * t + k3 ) * k2 * -1;
    
            case SpotGnc.refSine
                k1 = paramRefGen(phase,coord).k1;  % amplitude
                k2 = paramRefGen(phase,coord).k2;  % frequency
                k3 = paramRefGen(phase,coord).k3;  % phase
                k4 = paramRefGen(phase,coord).k4;  % offset
    
                ref(coord)     = k1 * sin( k2 * t + k3 ) + k4;
                ref_vel(coord) = k1 * cos( k2 * t + k3 ) * k2;
    
            case SpotGnc.refPolyWrap
                k1 = paramRefGen(phase,coord).k1;  % initial angle
                k2 = paramRefGen(phase,coord).k2;  % initial rate
    
                ref(coord)     = wrapToPi( k1 + k2 * t );
                ref_vel(coord) = k2;
    
            otherwise
                error('SpotRefGen.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function

