%% predeclare for code generation

numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structMeasProc.fun = SpotGnc.procNone;
structMeasProc.k1  = 0;
structMeasProc.k2  = 0;
structMeasProc.k3  = 0;
structMeasProc.k4  = 0;

paramMeasProc = repmat(structMeasProc,numPhase,numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector


%% SpotCoord.thetaRed - default

coord = SpotCoord.thetaRed;

for phase = allPhases
    paramMeasProc(phase,coord).fun = SpotGnc.procAngle;
end


%% SpotCoord.thetaBlack - default

coord = SpotCoord.thetaBlack;

for phase = allPhases
    paramMeasProc(phase,coord).fun = SpotGnc.procAngle;
end


%% SpotCoord.thetaBlue - default

coord = SpotCoord.thetaBlue;

for phase = allPhases
    paramMeasProc(phase,coord).fun = SpotGnc.procAngle;
end

