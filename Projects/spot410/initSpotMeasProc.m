%% predeclare for code generation

numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structMeasProc.fun = SpotGnc.procNone;
structMeasProc.k1  = 0;
structMeasProc.k2  = 0;
structMeasProc.k3  = 0;
structMeasProc.k4  = 0;

paramMeasProc = repmat(structMeasProc,numPhase,numCoord);

noiseMeas.mean = zeros(numCoord,1);
noiseMeas.seed = zeros(numCoord,1);
noiseMeas.var  = zeros(numCoord,1);

