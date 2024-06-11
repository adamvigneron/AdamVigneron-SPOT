function [F,diag] = SpotController(e, phase, coord, feedFwd, paramCtrl)
    
    %% persistent variables

    % definition of persistent variables
    persistent eOld;
    persistent eDeltaOld;

    % initialization of persistent variables
    if isempty(eOld)
        eOld = e;
        eDeltaOld = 0;
    end


    %% initialization of diagnostic output

    numPhase = size(paramCtrl,1); %#ok<NASGU>
    numCoord = size(paramCtrl,2);
    numDiag  = 3;

    diag = zeros(numDiag,numCoord);


    %% select a control method

    myFun = paramCtrl(phase,coord).fun;
    
    switch myFun

        case SpotGnc.ctrlNone

            F = 0;

        case SpotGnc.ctrlPd
            k1 = paramCtrl(phase,coord).k1;  % Kp
            k2 = paramCtrl(phase,coord).k2;  % Kd
            k3 = paramCtrl(phase,coord).k3;  % baseRate

            eDelta = e - eOld;

            if eDelta == 0
                eDelta = eDeltaOld;
            end

            % F = Kp*e + Kd*(de/dt)
            F = k1*e + k2*eDelta/k3;
            
            diag(1,coord) = k1*e;
            diag(2,coord) = k2*eDelta/k3;
            
            eOld      = e;
            eDeltaOld = eDelta;

        case SpotGnc.ctrlPdFwd
            k1 = paramCtrl(phase,coord).k1;  % Kp
            k2 = paramCtrl(phase,coord).k2;  % Kd
            k3 = paramCtrl(phase,coord).k3;  % baseRate
            k4 = paramCtrl(phase,coord).k4;  % beta

            eDelta = e - eOld;

            if eDelta == 0
                eDelta = eDeltaOld;
            end

            % F = Kp*e + Kd*(de/dt) + beta*uOld
            F = k1*e + k2*eDelta/k3 + k4*feedFwd(coord);

            diag(1,coord) = k1*e;
            diag(2,coord) = k2*eDelta/k3;
            diag(3,coord) = k4*feedFwd(coord);

            eOld      = e;
            eDeltaOld = eDelta;

         case SpotGnc.ctrlFwd
            F = feedFwd(coord);

        otherwise
            error('SpotController.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))

    end % switch myFun

end % function

