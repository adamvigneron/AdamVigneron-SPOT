%% predeclare for code generation

numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

paramEst = repmat(struct, numPhase, numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector

allCoords = enumeration('SpotCoord');
allCoords = allCoords(:).';  % converts to a row vector

phases1to6 = allPhases( (allPhases ~= SpotPhase.Phase0) );


%% reference orbit

rRef   = 0.85;  % radius, metres


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


%% SpotCoord.*Red - SpotPhase.Phase0

phase = SpotPhase.Phase0;

for coord = [ SpotCoord.xRed, SpotCoord.yRed, SpotCoord.thetaRed ]
    paramEst(phase,coord).fun = SpotGnc.estPolarStereo;
    paramEst(phase,coord).k1  = rRef;
    % paramEst(phase,coord).sensor has already been set
    % paramEst(phase,coord).rateSensor has already been set
end


%% SpotCoord.*Red - SpotPhase.Phase1 through Phase6

for phase = phases1to6
    for coord = [ SpotCoord.xRed, SpotCoord.yRed, SpotCoord.thetaRed ]
        paramEst(phase,coord).fun = SpotGnc.estEkfPolarStereo;
        paramEst(phase,coord).k1  = rRef;
        paramEst(phase,coord).k2  = baseRate;
        % paramEst(phase,coord).sensor has already been set
        % paramEst(phase,coord).rateSensor has already been set
    end
end

