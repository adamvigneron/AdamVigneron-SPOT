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
    fwdTemp = totCmd + mass * ( F \ ( G \ errStore(1:learnFactor:end,coord) ) );

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

