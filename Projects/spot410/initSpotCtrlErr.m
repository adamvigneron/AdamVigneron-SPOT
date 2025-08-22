%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structCtrlErr.fun = SpotGnc.errMinus;

paramCtrlErr = repmat(structCtrlErr,numPhase,numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector


%% SpotCoord.theta* - default

for phase = allPhases
    paramCtrlErr(phase,SpotCoord.thetaRed  ).fun = SpotGnc.errMinusWrap;
    paramCtrlErr(phase,SpotCoord.thetaBlack).fun = SpotGnc.errMinusWrap;
    paramCtrlErr(phase,SpotCoord.thetaBlue ).fun = SpotGnc.errMinusWrap;
end


%% SpotCoord.*Arm - default

for phase = allPhases
    paramCtrlErr(phase,SpotCoord.shoulderArm).fun = SpotGnc.errArmSetpoint;
    paramCtrlErr(phase,SpotCoord.elbowArm   ).fun = SpotGnc.errArmSetpoint;
    paramCtrlErr(phase,SpotCoord.wristArm   ).fun = SpotGnc.errArmSetpoint;
end

