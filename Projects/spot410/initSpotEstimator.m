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
    paramEst(phase,coord).k1  = baseRate;  % 2*baseRate in experiment
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
end


%% SpotCoord.yRed - default

% paramEst.fun is set to estNone by default


%% SpotCoord.thetaRed - default

% paramEst.fun is set to estNone by default


%% SpotCoord.xBlack - default

% paramEst.fun is set to estNone by default


%% SpotCoord.yBlack - default

% paramEst.fun is set to estNone by default


%% SpotCoord.thetaBlack - default

% paramEst.fun is set to estNone by default


%% SpotCoord.xBlue - default

% paramEst.fun is set to estNone by default


%% SpotCoord.yBlue - default

% paramEst.fun is set to estNone by default


%% SpotCoord.thetaBlue - default

% paramEst.fun is set to estNone by default


%% SpotCoord.shoulderArm - default

% paramEst.fun is set to estNone by default


%% SpotCoord.elbowArm - default

% paramEst.fun is set to estNone by default


%% SpotCoord.wristArm - default

% paramEst.fun is set to estNone by default

