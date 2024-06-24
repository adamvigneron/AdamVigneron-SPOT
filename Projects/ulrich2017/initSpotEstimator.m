%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structEst.fun = SpotGnc.estNone;
structEst.k1  = 0;
structEst.k2  = 0;
structEst.k3  = 0;

paramEst = repmat(structEst,numPhase,numCoord);


%% SpotPhase.Phase3_4 - SpotCoord.xRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.xRed;

paramEst(phase,coord).fun = SpotGnc.estVelBias;
paramEst(phase,coord).k1  = baseRate;
paramEst(phase,coord).k2  = 1/sqrt(10);  % L1
paramEst(phase,coord).k3  = 1/sqrt(10);  % L2
