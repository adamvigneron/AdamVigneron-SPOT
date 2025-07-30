%% predeclare for code generation

numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

paramEst = repmat(struct, numPhase, numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector

allCoords = enumeration('SpotCoord');
allCoords = allCoords(:).';  % converts to a row vector

phases3_1to3_4 = [SpotPhase.Phase3_1, SpotPhase.Phase3_2, ...
                  SpotPhase.Phase3_3, SpotPhase.Phase3_4];


%% by default, every coordinate uses its Phasespace/Encoder measurement and rate

for phase = allPhases
    for coord = allCoords
        paramEst(phase,coord).fun        = SpotGnc.estNone;
        paramEst(phase,coord).sensor     = SpotSensor(coord.real);
        paramEst(phase,coord).rateSensor = SpotSensor(numCoord + coord.real);
        paramEst(phase,coord).k1         = 0;
        paramEst(phase,coord).k2         = 0;
        paramEst(phase,coord).k3         = 0;
    end
end


%% SpotCoord.xRed - SpotPhase.Phase3_*

coord = SpotCoord.xRed;

for phase = phases3_1to3_4
    paramEst(phase,coord).fun = SpotGnc.estEkf3dof;
    paramEst(phase,coord).k1  = baseRate;
    % paramEst(phase,coord).sensor is set to phasespace by default
    % paramEst(phase,coord).rateSensor is set to phasespace by default
end


%% SpotCoord.yRed - SpotPhase.Phase3_*

coord = SpotCoord.yRed;

for phase = phases3_1to3_4
    paramEst(phase,coord).fun = SpotGnc.estEkf3dof;
    paramEst(phase,coord).k1  = baseRate;
    % paramEst(phase,coord).sensor is set to phasespace by default
    % paramEst(phase,coord).rateSensor is set to phasespace by default
end


%% SpotCoord.thetaRed - SpotPhase.Phase3_*

coord = SpotCoord.thetaRed;

for phase = phases3_1to3_4
    paramEst(phase,coord).fun = SpotGnc.estEkf3dof;
    paramEst(phase,coord).k1  = baseRate;
    % paramEst(phase,coord).sensor is set to phasespace by default
    % paramEst(phase,coord).rateSensor is set to phasespace by default
end


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

