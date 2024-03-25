%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structCtrlErr.fun = SpotKey.errMinus;

paramCtrlErr = repmat(structCtrlErr,numPhase,numCoord);


%% SpotPhase.Phase3_4 - SpotCoord.xRed

% paramCtrlErr.fun is set to errMinus by default


%% SpotPhase.Phase3_4 - SpotCoord.yRed

% paramCtrlErr.fun is set to errMinus by default


%% SpotPhase.Phase3_4 - SpotCoord.thetaRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.thetaRed;

paramCtrlErr(phase,coord).fun = SpotKey.errMinusWrap;

