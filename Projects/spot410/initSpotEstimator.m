%% predeclare for code generation

numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

paramEst = repmat(struct, numPhase, numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector

allCoords = enumeration('SpotCoord');
allCoords = allCoords(:).';  % converts to a row vector


%% by default, every coordinate uses its PhaseSpace measurement

for phase = allPhases
    for coord = allCoords
        paramEst(phase,coord).fun    = SpotGnc.estNone;
        paramEst(phase,coord).sensor = SpotSensor(coord.real);
        paramEst(phase,coord).k1     = 0;
        paramEst(phase,coord).k2     = 0;
        paramEst(phase,coord).k3     = 0;
    end
end


%% SpotCoord.xRed - default

coord = SpotCoord.xRed;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
    % paramEst(phase,coord).sensor already set
end


%% SpotCoord.yRed - default

coord = SpotCoord.yRed;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
    % paramEst(phase,coord).sensor already set
end


%% SpotCoord.thetaRed - default

coord = SpotCoord.thetaRed;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
    % paramEst(phase,coord).sensor already set
end


%% SpotCoord.xBlack - default

coord = SpotCoord.xBlack;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
    % paramEst(phase,coord).sensor already set
end


%% SpotCoord.yBlack - default

coord = SpotCoord.yBlack;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
    % paramEst(phase,coord).sensor already set
end


%% SpotCoord.thetaBlack - default

coord = SpotCoord.thetaBlack;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
    % paramEst(phase,coord).sensor already set
end


%% SpotCoord.xBlue - default

coord = SpotCoord.xBlue;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
    % paramEst(phase,coord).sensor already set
end


%% SpotCoord.yBlue - default

coord = SpotCoord.yBlue;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
    % paramEst(phase,coord).sensor already set
end


%% SpotCoord.thetaBlue - default

coord = SpotCoord.thetaBlue;

for phase = allPhases    
    paramEst(phase,coord).fun = SpotGnc.estVelBias;
    paramEst(phase,coord).k1  = baseRate;
    paramEst(phase,coord).k2  = 1;  % L1
    paramEst(phase,coord).k3  = 1;  % L2
    % paramEst(phase,coord).sensor already set
end


%% SpotCoord.shoulderArm - default

% paramEst.fun is set to estNone by default


%% SpotCoord.elbowArm - default

% paramEst.fun is set to estNone by default


%% SpotCoord.wristArm - default

% paramEst.fun is set to estNone by default


%% SpotPhase.Phase3_4 - SpotCoord.xRed|yRed|thetaRed

phase = SpotPhase.Phase3_4;

for coord = [SpotCoord.xRed SpotCoord.yRed SpotCoord.thetaRed]
    paramEst(phase,coord).fun = SpotGnc.estEkf3dof;
    paramEst(phase,coord).k1  = baseRate;
end


