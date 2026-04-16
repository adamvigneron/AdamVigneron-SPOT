function [F,debug] = SpotController(phase, err, err_vel, feedFwd, paramCtrl)

    %% initialization of output and persistent variables

    coords   = enumeration( SpotCoord(1) );
    phases   = enumeration( SpotPhase(1) );
    numCoord = length(coords);
    numPhase = length(phases);
    numDebug = 3;
    numStore = 2400;

    % output variables
    F     = zeros(numCoord,1);
    debug = zeros(numCoord,numDebug);
    
    % persistent variables - definition
    persistent errOld;
    persistent errDeltaOld;
    persistent cmdStore;
    persistent errStore;
    persistent fwdStore;
    persistent idxRW;
    persistent initFlag;
    persistent ilcF;
    persistent ilcG;

    % persistent variables - initialization
    if isempty(errOld)
        errOld = err;
        errDeltaOld = zeros(numCoord,1);
        cmdStore = zeros(numStore,3);
        errStore = zeros(numStore,3);
        fwdStore = zeros(numStore,3);
        idxRW    = zeros(numCoord,1);
        initFlag = zeros(numPhase,numCoord);

        [ilcF,ilcG] = initializeILC;
    end


    %% loop over all coordinates

    for k = 1:numCoord
        
        coord = coords(k);

        %% select a control method
    
        myFun = paramCtrl(phase,coord).fun;
        
        switch myFun
    
            case SpotGnc.ctrlNone
    
                F(coord) = 0;
    

            case SpotGnc.ctrlPd

                k1 = paramCtrl(phase,coord).k1;  % Kp
                k2 = paramCtrl(phase,coord).k2;  % Kd
                k3 = paramCtrl(phase,coord).k3;  % baseRate
    
                eDelta = err(coord) - errOld(coord);
    
                if eDelta == 0
                    eDelta = errDeltaOld(coord);
                end
    
                % F = Kp*e + Kd*(de/dt)
                F(coord) = k1*err(coord) + k2*eDelta/k3;

                debug(coord,1) = k1*err(coord);
                debug(coord,2) = k2*eDelta/k3;
                
                errOld(coord) = err(coord);
                errDeltaOld(coord) = eDelta;
    

            case { SpotGnc.ctrlPd_vel , SpotGnc.ctrlPd_vel_ilc }

                k1 = paramCtrl(phase,coord).k1;  % Kp
                k2 = paramCtrl(phase,coord).k2;  % Kd
    
                % F = Kp*e + Kd*(de/dt)
                F(coord) = k1*err(coord) + k2*err_vel(coord);
                
                debug(coord,1) = k1*err(coord);
                debug(coord,2) = k2*err_vel(coord);

                if myFun == SpotGnc.ctrlPd_vel_ilc

                    if phase == SpotPhase.Phase3_1

                        cmdStore( idxRW(coord) , coord ) = F(coord);
                        errStore( idxRW(coord) , coord ) = err(coord);
                        
                        idxRW(coord) = idxRW(coord) + 1;
               
                    elseif (phase == SpotPhase.Phase3_2) || (phase == SpotPhase.Phase3_3) || (phase == SpotPhase.Phase3_4)

                        if ~initFlag(phase,coord)
                            fwdStore = learning_control.initPtypeLearning(coord,errStore,cmdStore,fwdStore,ilcF,ilcG);
                            idxRW(coord) = 1;
                            initFlag(phase,coord) = 1;
                        end

                        cmdStore( idxRW(coord) , coord ) = F(coord);
                        errStore( idxRW(coord) , coord ) = err(coord);

                        F(coord) = F(coord) + fwdStore( idxRW(coord) , coord );
                        debug(coord,3)      = fwdStore( idxRW(coord) , coord );

                        idxRW(coord) = idxRW(coord) + 1;

                    end  % if phase
                
                end  % if myFun
    

            case SpotGnc.ctrlPdFwd
                k1 = paramCtrl(phase,coord).k1;  % Kp
                k2 = paramCtrl(phase,coord).k2;  % Kd
                k3 = paramCtrl(phase,coord).k3;  % baseRate
                k4 = paramCtrl(phase,coord).k4;  % beta
    
                eDelta = err(coord) - errOld(coord);
    
                if eDelta == 0
                    eDelta = errDeltaOld(coord);
                end
    
                % F = Kp*e + Kd*(de/dt) + beta*uOld
                F(coord) = k1*err(coord) + k2*eDelta/k3 + k4*feedFwd(coord);
    
                debug(coord,1) = k1*err(coord);
                debug(coord,2) = k2*eDelta/k3;
                debug(coord,3) = k4*feedFwd(coord);
    
                errOld(coord) = err(coord);
                errDeltaOld(coord) = eDelta;
    

            case SpotGnc.ctrlPdFwd_vel
                k1 = paramCtrl(phase,coord).k1;  % Kp
                k2 = paramCtrl(phase,coord).k2;  % Kd
                k4 = paramCtrl(phase,coord).k4;  % beta
    
                % F = Kp*e + Kd*(de/dt) + beta*uOld
                F(coord) = k1*err(coord) + k2*err_vel(coord) + k4*feedFwd(coord);
    
                debug(coord,1) = k1*err(coord);
                debug(coord,2) = k2*err_vel(coord);
                debug(coord,3) = k4*feedFwd(coord);
    

            case SpotGnc.ctrlArmSetpoint
                F(coord) = err(coord);
    
                debug(coord,1) = err(coord);
    

            otherwise
                error('SpotController.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function


function [F,G] = initializeILC()

    % PARAMETERS
    deployLength = 20;      % seconds
    baseRate     = 0.05;    % seconds
        
    % double integrator, continuous time
    % A = [0 1; 0 0];
    % B = [0; 1];
    % C = [1 0];
    % D = 0;
    
    % double integrator, discrete time (zoh)
    Ad = [1 baseRate; 0 1];
    Bd = [0.5*baseRate^2; baseRate];
    Cd = [1 0];
    % Dd = 0;
    
    % dimensions
    xDim   = size(Bd,1);
    uDim   = size(Bd,2);
    % yDim = size(Cd,1);
    
    % we only apply ILC during the manoeuvre itself
    nILC = round(deployLength / baseRate);
    
    % F matrix
    F_store        = zeros( xDim, uDim, nILC);
    F_store(:,:,2) = Bd;
    
    for q = 3:nILC
        F_store(:,:,q) = Ad * F_store(:,:,q-1);
    end
        
    F = zeros( nILC*xDim, nILC*uDim );
    
    for l = 1:nILC
        rowIdx = (1:xDim) + (l-1)*xDim;
    
        for m = 1:l
            colIdx = (1:uDim) + (m-1)*uDim;
            F(rowIdx,colIdx) = F_store(:,:,l-m+1);
        end
    end
    
    % G matrix
    GCell = repmat({Cd},1,nILC);
    G     = blkdiag(GCell{:});

end

