function [F,debug] = SpotController(phase, err, err_vel, feedFwd, paramCtrl)

    %% initialization of output and persistent variables

    coords   = enumeration( SpotCoord(1) );
    % phases   = enumeration( SpotPhase(1) );
    numCoord = length(coords);
    % numPhase = length(phases);
    numDebug = 3;
    % numStore = 2400;

    % output variables
    F     = zeros(numCoord,1);
    debug = zeros(numCoord,numDebug);
    
    % persistent variables - definition
    persistent errOld;
    persistent errDeltaOld;
    % persistent cmdStore;
    % persistent errStore;
    % persistent fwdStore;
    % persistent idxRW;
    % persistent initFlag;

    % persistent variables - initialization
    if isempty(errOld)
        errOld = err;
        errDeltaOld = zeros(numCoord,1);
        % cmdStore = zeros(numStore,3);
        % errStore = zeros(numStore,3);
        % fwdStore = zeros(numStore,3);
        % idxRW    =  ones(numCoord,1);
        % initFlag = zeros(numPhase,numCoord);
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

                % if myFun == SpotGnc.ctrlPd_vel_ilc
                % 
                %     if phase == SpotPhase.Phase3_1
                % 
                %         cmdStore( idxRW(coord) , coord ) = F(coord);
                %         errStore( idxRW(coord) , coord ) = err(coord);
                % 
                %         idxRW(coord) = idxRW(coord) + 1;
                % 
                %     elseif (phase == SpotPhase.Phase3_2) || (phase == SpotPhase.Phase3_3) || (phase == SpotPhase.Phase3_4)
                % 
                %         if ~initFlag(phase,coord)
                %             fwdStore = learning_control.initPtypeLearning(coord,errStore,cmdStore,fwdStore);
                %             idxRW(coord) = 1;
                %             initFlag(phase,coord) = 1;
                %         end
                % 
                %         cmdStore( idxRW(coord) , coord ) = F(coord);
                %         errStore( idxRW(coord) , coord ) = err(coord);
                % 
                %         F(coord) = F(coord) + fwdStore( idxRW(coord) , coord );
                %         debug(coord,3)      = fwdStore( idxRW(coord) , coord );
                % 
                %         idxRW(coord) = idxRW(coord) + 1;
                % 
                %     end  % if phase
                % 
                % end  % if myFun
    

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

