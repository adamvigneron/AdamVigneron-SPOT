%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structCtrlErr.fun = SpotGnc.errMinus;

paramCtrlErr = repmat(structCtrlErr,numPhase,numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector


%% SpotCoord.xRed - default

% paramCtrlErr.fun is set to errMinus by default


%% SpotCoord.yRed - default

% paramCtrlErr.fun is set to errMinus by default


%% SpotCoord.thetaRed - default

coord = SpotCoord.thetaRed;

for phase = allPhases
    paramCtrlErr(phase,coord).fun = SpotGnc.errMinusWrap;
end


%% SpotCoord.xBlack - default

% paramCtrlErr.fun is set to errMinus by default


%% SpotCoord.yBlack - default

% paramCtrlErr.fun is set to errMinus by default


%% SpotCoord.thetaBlack - default

coord = SpotCoord.thetaBlack;

for phase = allPhases
    paramCtrlErr(phase,coord).fun = SpotGnc.errMinusWrap;
end

%% SpotCoord.xBlue - default

% paramCtrlErr.fun is set to errMinus by default


%% SpotCoord.yBlue - default

% paramCtrlErr.fun is set to errMinus by default


%% SpotCoord.thetaBlue - default

coord = SpotCoord.thetaBlue;

for phase = allPhases
    paramCtrlErr(phase,coord).fun = SpotGnc.errMinusWrap;
end


%% SpotCoord.shoulderArm - default

coord = SpotCoord.shoulderArm;

for phase = allPhases
    paramCtrlErr(phase,coord).fun = SpotGnc.errArmSetpoint;
end


%% SpotCoord.elbowArm - default

coord = SpotCoord.elbowArm;

for phase = allPhases
    paramCtrlErr(phase,coord).fun = SpotGnc.errArmSetpoint;
end


%% SpotCoord.wristArm - default

coord = SpotCoord.wristArm;

for phase = allPhases
    paramCtrlErr(phase,coord).fun = SpotGnc.errArmSetpoint;
end

