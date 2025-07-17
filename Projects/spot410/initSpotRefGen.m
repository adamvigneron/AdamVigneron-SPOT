%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structRefGen.fun = SpotGnc.refConstant;
structRefGen.k1  = 0;
structRefGen.k2  = 0;
structRefGen.k3  = 0;
structRefGen.k4  = 0;
structRefGen.k5  = 0;

paramRefGen = repmat(structRefGen,numPhase,numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector


%% SpotPhase.* - Platforms

for phase = allPhases

    % paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant
    
    paramRefGen(phase,SpotCoord.xRed    ).k1 = drop_states_RED(1);
    paramRefGen(phase,SpotCoord.yRed    ).k1 = drop_states_RED(2);
    paramRefGen(phase,SpotCoord.thetaRed).k1 = drop_states_RED(3);
    
    paramRefGen(phase,SpotCoord.xBlack    ).k1 = drop_states_BLACK(1);
    paramRefGen(phase,SpotCoord.yBlack    ).k1 = drop_states_BLACK(2);
    paramRefGen(phase,SpotCoord.thetaBlack).k1 = drop_states_BLACK(3);
    
    paramRefGen(phase,SpotCoord.xBlue    ).k1 = drop_states_BLUE(1);
    paramRefGen(phase,SpotCoord.yBlue    ).k1 = drop_states_BLUE(2);
    paramRefGen(phase,SpotCoord.thetaBlue).k1 = drop_states_BLUE(3);

end


%% allPhases - SpotCoord.*Arm

for phase = [SpotPhase.Phase0   SpotPhase.Phase1   SpotPhase.Phase2 ...
             SpotPhase.Phase4   SpotPhase.Phase5   SpotPhase.Phase6]

    % paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant
    paramRefGen(phase,SpotCoord.shoulderArm).k1 =      pi/2;
    paramRefGen(phase,SpotCoord.elbowArm   ).k1 =      pi/2;
    paramRefGen(phase,SpotCoord.wristArm   ).k1 = -1 * pi/2;

end


%% SpotPhase.Phase3 - SpotCoord.*Arm 

for phase = [SpotPhase.Phase3_1  SpotPhase.Phase3_2 ...
             SpotPhase.Phase3_3  SpotPhase.Phase3_4 ]

    for coord = [SpotCoord.shoulderArm  SpotCoord.elbowArm  SpotCoord.wristArm]

        paramRefGen(phase,coord).fun = SpotGnc.refDeployStow;
        % paramRefGen(phase,coord).k1 is set for each coordinate individually
        % paramRefGen(phase,coord).k2 is set for each coordinate individually
        
        paramRefGen(phase,coord).k3 = paramPhaseMgmt(phase-1).phaseEnd;  % phase epoch
        paramRefGen(phase,coord).k4 = 20;  % deploy interval
        paramRefGen(phase,coord).k5 = 30;  % pause interval

    end

    paramRefGen(phase,SpotCoord.shoulderArm).k1 = pi/2;
    paramRefGen(phase,SpotCoord.shoulderArm).k2 = -1 * pi/3;

    paramRefGen(phase,SpotCoord.elbowArm).k1 = pi/2;
    paramRefGen(phase,SpotCoord.elbowArm).k2 = -1 * pi/3;

    paramRefGen(phase,SpotCoord.wristArm).k1 = -1 * pi/2;
    paramRefGen(phase,SpotCoord.wristArm).k2 = pi/6;

end

