function [est,est_vel,est_bias,debug] = SpotEstimator(phase, proc, cmd, paramEst)

    %% initialization of output and persistent variables

    coords      = enumeration( SpotCoord(1) );
    numCoord    = length(coords);
    maxEstState = 2;

    maxEkfState = 6;
    maxEkfMeas  = 3;

    numDebug = maxEkfState * ( maxEkfState + 1 );

    % output variables
    est      = zeros(numCoord,1);
    est_vel  = zeros(numCoord,1);
    est_bias = zeros(numCoord,1);
    debug    = zeros(numCoord,numDebug);
    
    % persistent variables - definition
    persistent estState;
    persistent prevEst;
    persistent measDelay;

    persistent ekfOutputPrev;
    persistent ekfP0;
    persistent ekfQ0;
    persistent ekfR0;

    % persistent variables - initialization
    if isempty(estState)
        estState  = zeros(maxEstState,numCoord);
        prevEst   = zeros(3,numCoord);
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

            case SpotGnc.estEkf3dof

                % for now, run the filter in open loop

                % position estimate is the processed measurement
                % velocity estimate is the measured rate
                % bias estimate remains at zero

                sensor = paramEst(phase,coord).sensor;
                est(coord) = proc(sensor);

                rateSensor = paramEst(phase,coord).rateSensor;
                est_vel(coord) = proc(rateSensor);

                % if the measurement hasn't changed, don't run the filter
                if est(coord) == prevEst(1,coord)
                    
                    debug(coord,:)   = ekfOutputPrev';
                    measDelay(coord) = measDelay(coord) + 1;

                else

                    switch coord

                        case { SpotCoord.yRed , SpotCoord.thetaRed }

                            % do nothing

                        case SpotCoord.xRed

                            k1 = paramEst(phase,coord).k1;  % baseRate

                            % time since last measurement
                            dtEkf = k1 * measDelay(coord);

                            % determine which sensors are available for processing
                            navigationSubmodule = 'navigation_module.EKF_rel_spot';
                            sensorModeStr       = [ navigationSubmodule '.select_sensor_mode'];
                            sensorModeFun       = str2func(sensorModeStr);
                            sensorMode          = sensorModeFun(proc);

                            switch sensorMode
                                case 'EKF_PhaseSpace'
                                    navigationSubmoduleMeas = [ navigationSubmodule '.EKF_PhaseSpace' ];
                                otherwise
                                    error('SpotEstimator.m:\n  sensor mode not defined')
                            end

                            % if needed, load the PQR matrices for the current EKF configuration
                            if ~any(ekfP0)
                                initializeEkfStr    = [ navigationSubmoduleMeas '.initialize_EKF' ];
                                initializeEkfFun    = str2func(initializeEkfStr);
                                [ekfP0,ekfQ0,ekfR0] = initializeEkfFun();
                            end

                            % if needed, initialize the EKF
                            if ~any(ekfOutputPrev)
                                initPos = [
                                    proc(SpotSensor.xBlackPhasespace)     - proc(SpotSensor.xRedPhasespace); ...
                                    proc(SpotSensor.yBlackPhasespace)     - proc(SpotSensor.yRedPhasespace); ...
                                    proc(SpotSensor.thetaBlackPhasespace) - proc(SpotSensor.thetaRedPhasespace)];
                                initVel = [ 0; 0; 0];
                                ekfOutputPrev = [
                                    initPos; ...
                                    initVel; ...
                                    reshape(ekfP0,[],1) ];
                            end

                            % query the EKF for a state estimate
                            queryEkfStr = [ navigationSubmodule '.query_ekf' ];
                            queryEkfFun = str2func(queryEkfStr);
                            ekfOutput   = queryEkfFun( sensorMode, ekfOutputPrev, proc, cmd, dtEkf, ekfQ0, ekfR0 );

                            % save filter output
                            ekfOutputPrev    = ekfOutput;
                            debug(coord,:)   = ekfOutput';

                            % update previous estimate and reset measurement delay
                            prevEst(1,coord) = est(coord);
                            measDelay(coord) = 1;

                        otherwise

                            error('SpotEstimator.m:\n  function SpotGnc.estEkf3dof not defined for SpotCoord(%d).\n\n', int32(coord))

                    end % switch coord

                end % if changed meas
                

            otherwise
                error('SpotEstimator.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function

