function [ref,ref_vel] = SpotRefGen(phase, t, proc, paramRefGen)

    %% initialization of output and persistent variables

    coords   = enumeration( SpotCoord(1) );
    numCoord = length(coords);

    ref     = zeros(numCoord,1);
    ref_vel = zeros(numCoord,1);

    % persistent variables - definition
    persistent xCtr;
    persistent yCtr;
    persistent thetaCorr;
    persistent xBlkRed_prev;

    % persistent variables - initialization
    if isempty(xCtr)
        xCtr         = 0;
        yCtr         = 0;
        thetaCorr    = 0;
        xBlkRed_prev = 0;
    end

    
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


            case SpotGnc.refHalfCosine

                k1 = paramRefGen(phase,coord).k1;  % initial value
                k2 = paramRefGen(phase,coord).k2;  % final value
                k3 = paramRefGen(phase,coord).k3;  % initial time
                k4 = paramRefGen(phase,coord).k4;  % final time

                A   = 0.5 * (k1 - k2);  % amplitude
                y0  = 0.5 * (k1 + k2);  % offset
                omg =  pi / (k4 - k3);  % angular frequency

                if t < k3
                    ref(coord) = k1;
                elseif t < k4
                    ref(coord) = A * cos( omg * (t - k3) ) + y0;
                else
                    ref(coord) = k2;
                end


            case SpotGnc.refDeployStow
                
                k1 = paramRefGen(phase,coord).k1;  % stow value
                k2 = paramRefGen(phase,coord).k2;  % deploy value
                k3 = paramRefGen(phase,coord).k3;  % phase epoch
                k4 = paramRefGen(phase,coord).k4;  % deploy interval
                k5 = paramRefGen(phase,coord).k5;  % pause interval

                A   = 0.5 * (k1 - k2);  % amplitude
                y0  = 0.5 * (k1 + k2);  % offset
                omg =  pi / k4;         % angular frequency

                t0 =      k3; % phase epoch
                t1 = t0 + k5; % start deploy
                t2 = t1 + k4; % end deploy
                t3 = t2 + k5; % start stow
                t4 = t3 + k4; % end stow

                if t < t1
                    ref(coord) = k1;
                elseif t < t2                    
                    ref(coord) = A * cos( omg * (t - t1) ) + y0;
                elseif t < t3
                    ref(coord) = k2;
                elseif t < t4
                    ref(coord) = A * cos( omg * (t4 - t) ) + y0;
                else
                    ref(coord) = k1;
                end


            case SpotGnc.refCircularInspection

                switch coord

                    case { SpotCoord.yRed , SpotCoord.thetaRed }

                        % do nothing

                    case SpotCoord.xRed

                        % first, we calculate the relative vector from RED to BLACK
                        
                        mySensor = paramRefGen(phase,coord).sensor;
                        
                        switch mySensor

                            case SpotSensor.xRedPhasespace
                                xBlkRed = proc(SpotSensor.xBlackPhasespace) - proc(SpotSensor.xRedPhasespace);
                                yBlkRed = proc(SpotSensor.yBlackPhasespace) - proc(SpotSensor.yRedPhasespace);

                            case SpotSensor.xStereo
                                xBody = proc(SpotSensor.xStereo);
                                yBody = proc(SpotSensor.yStereo);

                                thetaRed = proc(SpotSensor.thetaRedPhasespace);

                                xBlkRed = xBody * cos(thetaRed) - yBody * sin(thetaRed);
                                yBlkRed = yBody * cos(thetaRed) + xBody * sin(thetaRed);

                            case SpotSensor.xLidar
                                xBody = proc(SpotSensor.xLidar);
                                yBody = proc(SpotSensor.yLidar);

                                thetaRed = proc(SpotSensor.thetaRedPhasespace);

                                xBlkRed = xBody * cos(thetaRed) - yBody * sin(thetaRed);
                                yBlkRed = yBody * cos(thetaRed) + xBody * sin(thetaRed);

                            otherwise
                                error('SpotEstimator.m:\n  SpotSensor(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(mySensor), int32(phase), int32(coord));

                        end

                        % if this is a new measurement, update the inertial position of BLACK

                        if xBlkRed ~= xBlkRed_prev

                                xCtrRed = xCtr - proc(SpotSensor.xRedPhasespace);
                                yCtrRed = yCtr - proc(SpotSensor.yRedPhasespace);

                                % correct the phase of the inspection orbit to account for the  
                                % difference between the prior orbit centre and the updated one
                                thetaBlkRed = atan2(yBlkRed,xBlkRed);
                                thetaCtrRed = atan2(yCtrRed,xCtrRed);
                                thetaCorr   = thetaBlkRed - thetaCtrRed;

                                % update inertial position of BLACK
                                xCtr = xBlkRed + proc(SpotSensor.xRedPhasespace);
                                yCtr = yBlkRed + proc(SpotSensor.yRedPhasespace);
                                
                                % save new measurement for future comparison
                                xBlkRed_prev = xBlkRed;

                        end

                        k1 = paramRefGen(phase,coord).k1;  % amplitude
                        k2 = paramRefGen(phase,coord).k2;  % frequency
                        k3 = paramRefGen(phase,coord).k3;  % translational phase
                        k4 = paramRefGen(phase,coord).k4;  % rotational phase

                        ref(SpotCoord.xRed)         = k1 * cos( k2 * t + k3 + thetaCorr ) + xCtr;
                        ref(SpotCoord.yRed)         = k1 * sin( k2 * t + k3 + thetaCorr ) + yCtr;
                        ref(SpotCoord.thetaRed)     = wrapToPi( k2 * t + k4 + thetaCorr );

                        ref_vel(SpotCoord.xRed)     = k1 * sin( k2 * t + k3 + thetaCorr ) * k2 * -1;
                        ref_vel(SpotCoord.yRed)     = k1 * cos( k2 * t + k3 + thetaCorr ) * k2;
                        ref_vel(SpotCoord.thetaRed) = k2;

                        
                    otherwise
                        error('SpotEstimator.m:\n  function SpotGnc.estEkf3dof not defined for SpotCoord(%d).\n\n', int32(coord))

                end % switch coord

            otherwise
                error('SpotRefGen.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))

        end % switch myFun

    end % loop coords

end % function

