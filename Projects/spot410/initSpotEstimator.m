%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structEst.fun = SpotGnc.estNone;
structEst.k1  = 0;
structEst.k2  = 0;
structEst.k3  = 0;

paramEst = repmat(structEst,numPhase,numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector


%% SpotCoord.xRed - default

coord = SpotCoord.xRed;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.yRed - default

coord = SpotCoord.yRed;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.thetaRed - default

coord = SpotCoord.thetaRed;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.xBlack - default

coord = SpotCoord.xBlack;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.yBlack - default

coord = SpotCoord.yBlack;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.thetaBlack - default

coord = SpotCoord.thetaBlack;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.xBlue - default

coord = SpotCoord.xBlue;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.yBlue - default

coord = SpotCoord.yBlue;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.thetaBlue - default

coord = SpotCoord.thetaBlue;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.shoulderArm - default

% paramEst.fun is set to estNone by default


%% SpotCoord.elbowArm - default

% paramEst.fun is set to estNone by default


%% SpotCoord.wristArm - default

% paramEst.fun is set to estNone by default

