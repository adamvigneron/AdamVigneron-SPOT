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


%% SpotSensor.theta*Phasespace - default

for phase = allPhases
    paramMeasProc(phase,SpotSensor.thetaRedPhasespace  ).fun = SpotGnc.procAngle;
    paramMeasProc(phase,SpotSensor.thetaBlackPhasespace).fun = SpotGnc.procAngle;
    paramMeasProc(phase,SpotSensor.thetaBluePhasespace ).fun = SpotGnc.procAngle;
end


%% SpotCoord.thetaRedImu

sensor = SpotSensor.thetaRedImu;

for phase = allPhases
    paramMeasProc(phase,sensor).fun = SpotGnc.procImuPhasespace;
end

% % time constant of the exponential moving average
% tau = 3;  % sec
% 
% for phase = allPhases
%     paramMeasProc(phase,sensor).fun = SpotGnc.procImuBias;
%     paramMeasProc(phase,sensor).k1  = tau;
%     paramMeasProc(phase,sensor).k2  = baseRate;
% end


%% SpotCoord.thetaStereo | SpotCoord.thetaLidar - default

for phase = allPhases
    paramMeasProc(phase,SpotSensor.thetaStereo).fun = SpotGnc.procAngleQuadrant;
    paramMeasProc(phase,SpotSensor.thetaLidar ).fun = SpotGnc.procAngleQuadrant;
end

