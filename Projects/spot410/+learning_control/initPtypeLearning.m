function fwdStore = initPtypeLearning(coord,errStore,cmdStore,fwdStore,ilcF,ilcG)
    
    % PARAMETERS
    mRED         = 11.297;  % mass for RED, kg
    IRED         = 0.1982;  % MoI for RED, kg.m2
    deployLength = 15;      % seconds
    pauseLength  = 35;      % seconds
    baseRate     = 0.05;    % seconds
    aveWindow    = 1;       % seconds
    cmdFactor    = 0.5;     % unitless on the range 0 to 1 inclusive

    % assemble mass properties for RED
    mass = [mRED mRED IRED];
        
    % we preserve all of the feedforward and part of the feedback
    totCmd = cmdFactor * cmdStore(1:end,coord) ...
                       + fwdStore(1:end,coord);

    % determine the vector indices corresponding to arm movement
    deployStartIdx  = 1               + round( pauseLength / baseRate);
    deployEndIdx    = deployStartIdx  + round(deployLength / baseRate);
    retractStartIdx = deployEndIdx    + round( pauseLength / baseRate);
    retractEndIdx   = retractStartIdx + round(deployLength / baseRate);

    depRge = deployStartIdx :(deployEndIdx-1);
    retRge = retractStartIdx:(retractEndIdx-1);

    errDep = errStore(depRge,coord) - errStore(deployStartIdx, coord);
    errRet = errStore(retRge,coord) - errStore(retractStartIdx,coord);

    % calculate the new feedforward using the lifted vector method
    fwdTemp         = 0 * totCmd;
    fwdTemp(depRge) = totCmd(depRge) + mass(coord) * ( ilcF \ ( ilcG \ errDep ) );
    fwdTemp(retRge) = totCmd(retRge) + mass(coord) * ( ilcF \ ( ilcG \ errRet ) );

    % zero-order hold the endpoints (a lifted-vector bug)
    fwdTemp(deployStartIdx)  = fwdTemp(deployStartIdx+1);
    fwdTemp(deployEndIdx)    = fwdTemp(deployEndIdx-1);
    fwdTemp(retractStartIdx) = fwdTemp(retractStartIdx+1);
    fwdTemp(retractEndIdx)   = fwdTemp(retractEndIdx-1);

    % smooth the feedforward using a running average
    avePts = 2*round(aveWindow / baseRate) + 1;
    fwdStore(depRge,coord) = movmean(fwdTemp(depRge),avePts);
    fwdStore(retRge,coord) = movmean(fwdTemp(retRge),avePts);

    % zero the feed-forward when the arm is not moving
    fwdStore(             1:deployStartIdx  , coord ) = 0;
    fwdStore(  deployEndIdx:retractStartIdx , coord ) = 0;
    fwdStore( retractEndIdx:end             , coord ) = 0;

end

