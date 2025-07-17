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

    % persistent variables - initialization
    if isempty(errOld)
        errOld = err;
        errDeltaOld = zeros(numCoord,1);
        cmdStore = zeros(numStore,3);
        errStore = zeros(numStore,3);
        fwdStore = zeros(numStore,3);
        idxRW    = zeros(numCoord,1);
        initFlag = zeros(numPhase,numCoord);
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
                            fwdStore = initPtypeLearning(coord,errStore,cmdStore,fwdStore);
                            idxRW(coord) = 1;
                            initFlag(phase,coord) = 1;
                        end

                        F(coord) = F(coord) + fwdStore( idxRW(coord) , coord );
                        debug(coord,3)      = fwdStore( idxRW(coord) , coord );

                        cmdStore( idxRW(coord) , coord ) = F(coord);
                        errStore( idxRW(coord) , coord ) = err(coord);

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


function fwdStore = initPtypeLearning(coord,errStore,cmdStore,fwdStore)
    
    % PARAMETERS
    mass         = 0.1982;  % MoI for RED, kg.m2
    deployLength = 20;      % seconds
    pauseLength  = 30;      % seconds
    baseRate     = 0.05;    % seconds
    aveWindow    = 1;       % seconds
    cmdFactor    = 0.5;     % unitless on the range 0 to 1 inclusive
    learnFactor  = 2;       % unitless nonzero positive integer
    
    % we run the learning algorithm two times slower than the model
    learnRate   = learnFactor * baseRate;  % seconds
    
    % double integrator, continuous time
    % A = [0 1; 0 0];
    % B = [0; 1];
    % C = [1 0];
    % D = 0;
    
    % double integrator, discrete time (zoh)
    Ad = [1 learnRate; 0 1];
    Bd = [0.5*learnRate^2; learnRate];
    Cd = [1 0];
    % Dd = 0;
    
    % dimensions
    xDim   = size(Bd,1);
    uDim   = size(Bd,2);
    % yDim = size(Cd,1);
    
    nStore = length( fwdStore(:,coord) );
    nLearn = ceil( nStore / learnFactor );
    
    % F matrix
    F_store        = zeros( xDim, uDim, nLearn);
    F_store(:,:,2) = Bd;
    
    for q = 3:nLearn
        F_store(:,:,q) = Ad * F_store(:,:,q-1);
    end
        
    F = zeros( nLearn*xDim, nLearn*uDim );
    
    for l = 1:nLearn
        rowIdx = (1:xDim) + (l-1)*xDim;
    
        for m = 1:l
            colIdx = (1:uDim) + (m-1)*uDim;
            F(rowIdx,colIdx) = F_store(:,:,l-m+1);
        end
    end
    
    % G matrix
    GCell = repmat({Cd},1,nLearn);
    G     = blkdiag(GCell{:});

    % we preserve all of the feedforward and part of the feedback
    totCmd = cmdFactor * cmdStore(1:learnFactor:end,coord) ...
                       + fwdStore(1:learnFactor:end,coord);

    % calculate the new feedforward using the lifted vector method
    fwdTemp = totCmd + mass * F \ ( G \ errStore(1:learnFactor:end,coord) );

    % smooth the feedforward using a running average
    avePts  = 2*round(aveWindow / learnRate) + 1;
    fwdTemp = movmean(fwdTemp,avePts);

    % zero-order-hold the feedforward according to the learnFactor
    for i = 1:nStore
        fwdStore(i,coord) = fwdTemp( ceil(i / learnFactor) );
    end

    % determine the vector indices corresponding to arm movement
    deployStartIdx  = 1               + round( pauseLength / baseRate);
    deployEndIdx    = deployStartIdx  + round(deployLength / baseRate);
    retractStartIdx = deployEndIdx    + round( pauseLength / baseRate);
    retractEndIdx   = retractStartIdx + round(deployLength / baseRate);

    % zero the feed-forward when the arm is not moving
    fwdStore(             1:deployStartIdx  , coord ) = 0;
    fwdStore(  deployEndIdx:retractStartIdx , coord ) = 0;
    fwdStore( retractEndIdx:end             , coord ) = 0;

end