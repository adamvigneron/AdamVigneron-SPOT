%% predeclare for code generation

numPhase  = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numSensor = length(meta.class.fromName('SpotSensor').EnumerationMemberList);

structMeasProc.fun = SpotGnc.procNone;

paramMeasProc = repmat(structMeasProc,numPhase,numSensor);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector


%% SpotSensor.thetaRedPhasespace - default

sensor = SpotSensor.thetaRedPhasespace;

for phase = allPhases
    paramMeasProc(phase,sensor).fun = SpotGnc.procAngle;
end


%% SpotCoord.thetaBlackPhasespace - default

sensor = SpotSensor.thetaBlackPhasespace;

for phase = allPhases
    paramMeasProc(phase,sensor).fun = SpotGnc.procAngle;
end


%% SpotCoord.thetaBluePhasespace - default

sensor = SpotSensor.thetaBluePhasespace;

for phase = allPhases
    paramMeasProc(phase,sensor).fun = SpotGnc.procAngle;
end

