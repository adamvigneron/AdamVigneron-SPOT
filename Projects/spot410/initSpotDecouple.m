%% predeclare for code generation

numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structDcpl.fun = SpotGnc.dcplSingleAxis;
structDcpl.k1  = 0;

paramDcpl = repmat(structDcpl,numPhase,numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector

phases2to5 = allPhases( (allPhases ~= SpotPhase.Phase0) & ...
                        (allPhases ~= SpotPhase.Phase1) & ...
                        (allPhases ~= SpotPhase.Phase6) );


%% Platforms - default

% structDcpl(phase,coord).fun already set to SpotGnc.dcplSingleAxis

for phase = allPhases
    paramDcpl(phase,SpotCoord.xRed       ).k1 = mRED;
    paramDcpl(phase,SpotCoord.yRed       ).k1 = mRED;
    paramDcpl(phase,SpotCoord.thetaRed   ).k1 = IRED;
    paramDcpl(phase,SpotCoord.xBlack     ).k1 = mBLACK;
    paramDcpl(phase,SpotCoord.yBlack     ).k1 = mBLACK;
    paramDcpl(phase,SpotCoord.thetaBlack ).k1 = IBLACK;
    paramDcpl(phase,SpotCoord.xBlue      ).k1 = mBLUE;
    paramDcpl(phase,SpotCoord.yBlue      ).k1 = mBLUE;
    paramDcpl(phase,SpotCoord.thetaBlue  ).k1 = IBLUE;
end


%% SpotCoord.xRed | SpotCoord.yRed - SpotPhase.Phase2 to SpotPhase.Phase5

for phase = phases2to5
    for coord = [ SpotCoord.xRed, SpotCoord.yRed]
        paramDcpl(phase,coord).fun = SpotGnc.dcplRedBodyForce;
        % paramDcpl(phase,coord).k1 already set to mRED
    end
end


%% SpotCoord.thetaRed - SpotPhase.Phase2 to SpotPhase.Phase5

coord = SpotCoord.thetaRed;

for phase = phases2to5
    paramDcpl(phase,coord).fun = SpotGnc.dcplSingleAxisInvert;
    % paramDcpl(phase,coord).k1 already set to IRED
end


%% SpotCoord.*Arm - default

for phase = allPhases
    for coord = [ SpotCoord.shoulderArm , SpotCoord.elbowArm, SpotCoord.wristArm ]
        paramDcpl(phase,coord).fun = SpotGnc.dcplArmSetpoint;
    end
end
    

