function [est,est_vel,est_bias] = SpotEstimator(phase, proc, cmd, paramEst)

    %% initialization of output and persistent variables

    coords      = enumeration('SpotCoord');
    numCoord    = length(coords);
    maxEstState = 2;

    % output variables
    est      = proc;
    est_vel  = zeros(numCoord,1);
    est_bias = zeros(numCoord,1);
    
    % persistent variables - definition
    persistent estState;
    persistent prevEst;

    % persistent variables - initialization
    if isempty(estState)
        estState = zeros(maxEstState,numCoord);
        prevEst  = zeros(3,numCoord);
    end


    %% loop over all coordinates

    for k = 1:numCoord
        
        coord = coords(k);

        %% select an estimation method
    
        myFun = paramEst(phase,coord).fun;
        
        switch myFun
    
            case SpotGnc.estNone
                % position estimate is the processed measurement
                % velocity estimate remains at zero
                % bias estimate remains at zero
    
            case SpotGnc.estVelBias
                % position estimate is the processed measurement
                % velocity and bias estimate use reduced-order observer 
                % (for the double-integrator plant)

                k1 = paramEst(phase,coord).k1;  % baseRate
                k2 = paramEst(phase,coord).k2;  % L1
                k3 = paramEst(phase,coord).k3;  % L2

                % observer gains
                L1 = k2;
                L2 = k3;

                % continuous time
                A = [ -L1,       1; -L2,      0];
                B = [   1, L2-L1^2;   0, -L1*L2];

                % discrete time
                Ad = expm(A*k1);
                Bd = A \ (Ad - eye(2)) * B;
                Cd = [1  0; 0  1];
                Dd = [0 L1; 0 L2];

                % if this is the first state estimate,
                % initialize the observer at zero initial velocity
                if ~ any( estState(:,coord) )
                    estState(:,coord) = -1 * [L1; L2] * proc(coord);
                end

                % if the measurement hasn't changed, don't run the observer
                if proc(coord) == prevEst(1,coord)
                    
                    est_vel(coord)  = prevEst(2,coord);
                    est_bias(coord) = prevEst(3,coord);
                
                else

                    % run the observer in discrete time
                    rHat              = Cd * estState(:,coord) + Dd * [cmd(coord); proc(coord)];
                    estState(:,coord) = Ad * estState(:,coord) + Bd * [cmd(coord); proc(coord)];

                    % map the discrete-time output onto the function output
                    est_vel(coord)  = rHat(1);
                    est_bias(coord) = rHat(2);

                    % save the observer output for next time
                    prevEst(1,coord) = proc(coord);
                    prevEst(2,coord) = est_vel(coord);
                    prevEst(3,coord) = est_bias(coord);

                end
                
            otherwise
                error('SpotEstimator.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function

