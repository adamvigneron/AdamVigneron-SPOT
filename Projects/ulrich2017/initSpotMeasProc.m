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


%% SpotPhase.Phase3_4 - SpotCoord.xRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.xRed;

paramMeasProc(phase,coord).fun = SpotGnc.procFreeze;
paramMeasProc(phase,coord).k1  = 0.010;   % position predicton interval
paramMeasProc(phase,coord).k2  = 0.0125;  % velocity calculation interval
paramMeasProc(phase,coord).k3  = 10;      % velocity rejection threshold
paramMeasProc(phase,coord).k4  = 10;      % repeated measurement threshold

