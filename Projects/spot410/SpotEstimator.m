function [est,est_vel,est_bias,debug] = SpotEstimator(phase, proc, cmd, paramEst)

    %% initialization of output and persistent variables

    coords      = enumeration( SpotCoord(1) );
    numCoord    = length(coords);
    maxEstState = 2;

    maxEkfState = 7;
    maxEkfMeas  = 4;

    numDebug = maxEkfState * ( maxEkfState + 1 );

    % output variables
    est      = zeros(numCoord,1);
    est_vel  = zeros(numCoord,1);
    est_bias = zeros(numCoord,1);
    debug    = zeros(numCoord,numDebug);
    
    % persistent variables - definition
    persistent estState;
    persistent prevEst;
    persistent prevPose;
    persistent measDelay;

    persistent ekfOutputPrev;
    persistent ekfP0;
    persistent ekfQ0;
    persistent ekfR0;

    % persistent variables - initialization
    if isempty(estState)
        estState  = zeros(maxEstState,numCoord);
        prevEst   = zeros(3,numCoord);
        prevPose  = zeros(3,numCoord);
        measDelay = ones(1,numCoord);

        ekfOutputPrev = zeros(    numDebug, 1);
        ekfP0         = zeros( maxEkfState, maxEkfState  );
        ekfQ0         = zeros( maxEkfState, maxEkfState );
        ekfR0         = zeros(  maxEkfMeas, maxEkfMeas );
    end


    %% loop over all coordinates

    for k = 1:numCoord
        
        coord = coords(k);

        %% select an estimation method
    
        myFun = paramEst(phase,coord).fun;
        
        switch myFun
    
            case SpotGnc.estNone
                % position estimate is the processed measurement
                % velocity estimate is the measured rate
                % bias estimate remains at zero

                sensor = paramEst(phase,coord).sensor;
                est(coord) = proc(sensor);

                rateSensor = paramEst(phase,coord).rateSensor;
                est_vel(coord) = proc(rateSensor);

                
            case SpotGnc.estVelBias
                % position estimate is the processed measurement
                % velocity and bias estimate use reduced-order observer 
                % (for the double-integrator plant)

                sensor = paramEst(phase,coord).sensor;
                est(coord) = proc(sensor);
                
                k1 = paramEst(phase,coord).k1;  % baseRate
                k2 = paramEst(phase,coord).k2;  % L1
                k3 = paramEst(phase,coord).k3;  % L2

                % observer gains
                L1 = k2;
                L2 = k3;

                % continuous time
                A = [ -L1,       1; -L2,      0];
                B = [   1, L2-L1^2;   0, -L1*L2];

                % time since last measurement
                dt = k1 * measDelay(coord);  % time since last measurement

                % discrete time
                Ad = expm(A*dt);
                Bd = A \ (Ad - eye(2)) * B;
                Cd = [1  0; 0  1];
                Dd = [0 L1; 0 L2];

                % if this is the first state estimate,
                % initialize the observer at zero initial velocity
                if ~ any( estState(:,coord) )
                    estState(:,coord) = -1 * [L1; L2] * proc(sensor);
                end

                % if the measurement hasn't changed, don't run the observer
                if proc(sensor) == prevEst(1,coord)
                    
                    est_vel(coord)  = prevEst(2,coord);
                    est_bias(coord) = prevEst(3,coord);

                    measDelay(coord) = measDelay(coord) + 1;
                
                else

                    % run the observer in discrete time
                    rHat              = Cd * estState(:,coord) + Dd * [cmd(coord); proc(sensor)];
                    estState(:,coord) = Ad * estState(:,coord) + Bd * [cmd(coord); proc(sensor)];

                    % map the discrete-time output onto the function output
                    est_vel(coord)  = rHat(1);
                    est_bias(coord) = rHat(2);

                    % save the observer output for next time
                    prevEst(1,coord) = proc(sensor);
                    prevEst(2,coord) = est_vel(coord);
                    prevEst(3,coord) = est_bias(coord);

                    measDelay(coord) = 1;

                end

            case { SpotGnc.estEkfStereo , SpotGnc.estEkfLidar, SpotGnc.estEkfPolarStereo, SpotGnc.estEkfPolarLidar }

                % we only run the filter for SpotCoord.xRed
                switch coord

                    case { SpotCoord.yRed , SpotCoord.thetaRed }

                        % do nothing

                    case SpotCoord.xRed

                        rRef     = paramEst(phase,coord).k1;
                        baseRate = paramEst(phase,coord).k2;

                        % if needed, load the PQR matrices for the current EKF configuration
                        if ~any(ekfP0)
                            [ekfP0,ekfQ0,ekfR0] = navigation_module.EKF_rel_spot.initialize_EKF( ...
                                baseRate, myFun );
                        end

                        % assemble the current pose measurement
                        switch myFun
                            case { SpotGnc.estEkfStereo, SpotGnc.estEkfPolarStereo }
                                procPose = [ proc(SpotSensor.xStereo); 
                                             proc(SpotSensor.yStereo); 
                                             proc(SpotSensor.thetaStereo) ];
                            case { SpotGnc.estEkfLidar , SpotGnc.estEkfPolarLidar }
                                procPose = [ proc(SpotSensor.xLidar);
                                             proc(SpotSensor.yLidar); 
                                             proc(SpotSensor.thetaLidar) ];
                            otherwise
                                error('SpotEstimator.m:\n  sensor not defined for relative pose')
                        end

                        % if needed, initialize the EKF output
                        if ~any(ekfOutputPrev)
                            ekfOutputPrev = [ procPose;                      % position
                                              [0; 0; 0];                     % velocity
                                              proc(SpotSensor.thetaRedImu);  % omega
                                              reshape(ekfP0,[],1) ];
                        end

                        % propagate state estimates from previous time step to a-priori estimates
                        ekfOutput = navigation_module.EKF_rel_spot.propagation( ...
                            ekfOutputPrev, cmd, baseRate, ekfQ0 );

                        % if measurements are available, correct to a-posteriori estimates
                        if norm( procPose - prevPose(:,coord) ) < 1e-10
                            % do nothing
                        else
                            measVec   = [procPose; proc(SpotSensor.thetaRedImu)];
                            ekfOutput = navigation_module.EKF_rel_spot.correction( ...
                                ekfOutput, measVec, ekfR0 );
                        end

                        % save filter output
                        ekfOutputPrev  = ekfOutput;
                        debug(coord,:) = ekfOutput';

                        % update previous pose measurement
                        prevPose(:,coord) = procPose;

                        if ( myFun == SpotGnc.estEkfPolarStereo ) || ( myFun == SpotGnc.estEkfPolarLidar )
                            
                            % xEst    = ekfOutput(1);
                            % yEst    = ekfOutput(2);
                            % xDotEst = ekfOutput(4);
                            % yDotEst = ekfOutput(5);
                            % 
                            % range     = sqrt( xEst^2 + yEst^2 );
                            % bearing   = atan( yEst   / xEst   );  % assume positive xEst
                            % 
                            % rangeDot   = ( xEst * xDotEst + yEst * yDotEst ) / range;
                            % bearingDot = ( xEst * yDotEst - yEst * xDotEst ) / range^2;
                            % 
                            % arcLength    = rRef * proc(SpotSensor.thetaRedPhasespace);
                            % arcLengthDot = rRef * proc(SpotSensor.thetaRedRatePhasespace);
                            % 
                            % est(SpotCoord.xRed)     = range;
                            % est(SpotCoord.yRed)     = arcLength;
                            % est(SpotCoord.thetaRed) = bearing;
                            % 
                            % est_vel(SpotCoord.xRed)     = rangeDot;
                            % est_vel(SpotCoord.yRed)     = arcLengthDot;
                            % est_vel(SpotCoord.thetaRed) = bearingDot;

                            range     = sqrt ( procPose(1)^2 + procPose(2)^2 );
                            bearing   = atan2( procPose(2)   , procPose(1)   );
                            arcLength = rRef * proc(SpotSensor.thetaRedPhasespace);

                            est(SpotCoord.xRed)     = range;
                            est(SpotCoord.yRed)     = arcLength;
                            est(SpotCoord.thetaRed) = bearing;

                            % velocity and bias estimates remain at zero

                        else

                            % output estimates for xRed and yRed
                            est(SpotCoord.xRed)         = ekfOutput(1);
                            est(SpotCoord.yRed)         = ekfOutput(2);
                            est_vel(SpotCoord.xRed)     = ekfOutput(4);
                            est_vel(SpotCoord.yRed)     = ekfOutput(5);
    
                            % theta estimates remain inertial
                            est(SpotCoord.thetaRed) = proc( ...
                                paramEst(phase,SpotCoord.thetaRed).sensor);
                            est_vel(SpotCoord.thetaRed) = proc( ...
                                paramEst(phase,SpotCoord.thetaRed).rateSensor);

                        end
                        
                        % bias estimates remain at zero

                    otherwise
                        error('SpotEstimator.m:\n  function SpotGnc.estEkf3dof not defined for SpotCoord(%d).\n\n', int32(coord))

                end % switch coord


            case { SpotGnc.estPolarStereo, SpotGnc.estPolarLidar }

                % we only assign measurements for SpotCoord.xRed
                switch coord

                    case { SpotCoord.yRed , SpotCoord.thetaRed }

                        % do nothing

                    case SpotCoord.xRed

                        rRef = paramEst(phase,coord).k1;

                        % assemble the current pose measurement
                        switch myFun
                            case SpotGnc.estPolarStereo
                                procPose = [ proc(SpotSensor.xStereo); 
                                             proc(SpotSensor.yStereo); 
                                             proc(SpotSensor.thetaStereo) ];
                            case SpotGnc.estPolarLidar
                                procPose = [ proc(SpotSensor.xLidar);
                                             proc(SpotSensor.yLidar); 
                                             proc(SpotSensor.thetaLidar) ];
                            otherwise
                                error('SpotEstimator.m:\n  sensor not defined for relative pose')
                        end

                        range     = sqrt ( procPose(1)^2 + procPose(2)^2 );
                        bearing   = atan2( procPose(2)   , procPose(1)   );
                        arcLength = rRef * proc(SpotSensor.thetaRedPhasespace);

                        est(SpotCoord.xRed)     = range;
                        est(SpotCoord.yRed)     = arcLength;
                        est(SpotCoord.thetaRed) = bearing;

                        % velocity and bias estimates remain at zero


                    otherwise
                        error('SpotEstimator.m:\n  function SpotGnc.estPolar not defined for SpotCoord(%d).\n\n', int32(coord))
                end


            otherwise
                error('SpotEstimator.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function

