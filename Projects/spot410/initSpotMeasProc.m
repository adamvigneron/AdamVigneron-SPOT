%% predeclare for code generation

numPhase  = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numSensor = length(meta.class.fromName('SpotSensor').EnumerationMemberList);

structMeasProc.fun = SpotGnc.procNone;
structMeasProc.k1  = 0;
structMeasProc.k2  = 0;

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


%% SpotCoord.thetaRedImu

sensor = SpotSensor.thetaRedImu;

% time constant of the exponential moving average
tau = 3;  % sec

for phase = allPhases
    paramMeasProc(phase,sensor).fun = SpotGnc.procImuBias;
    paramMeasProc(phase,sensor).k1  = tau;
    paramMeasProc(phase,sensor).k2  = baseRate;
end


%% SpotCoord.thetaStereo - default

sensor = SpotSensor.thetaStereo;

for phase = allPhases
    paramMeasProc(phase,sensor).fun = SpotGnc.procAngleQuadrant;
end


%% SpotCoord.thetaLidar - default

sensor = SpotSensor.thetaLidar;

for phase = allPhases
    paramMeasProc(phase,sensor).fun = SpotGnc.procAngleQuadrant;
end
