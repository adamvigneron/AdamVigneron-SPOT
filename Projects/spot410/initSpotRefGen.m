%% predeclare for code generation

numPhase = length( enumeration( SpotPhase(1) ) );
numCoord = length( enumeration( SpotCoord(1) ) );

structRefGen.fun = SpotGnc.refConstant;
structRefGen.k1  = 0;
structRefGen.k2  = 0;
structRefGen.k3  = 0;
structRefGen.k4  = 0;
structRefGen.k5  = 0;

paramRefGen = repmat(structRefGen, numPhase, numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector

phases3_1to3_4 = [SpotPhase.Phase3_1, SpotPhase.Phase3_2, ...
                  SpotPhase.Phase3_3, SpotPhase.Phase3_4];


%% reference orbit

rRef   = 0.85;  % radius, metres
omgRef = 0.03490659;  % angular frequency, rad/s

% we define a phase offset corresponding to the orbit start time
startPhase = omgRef * Phase2_End;


%% SpotCoord.*Red - default

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

for phase = allPhases
    paramRefGen(phase,SpotCoord.xRed    ).k1 = rRef;
    paramRefGen(phase,SpotCoord.yRed    ).k1 = rRef * -1 * pi;
    paramRefGen(phase,SpotCoord.thetaRed).k1 = 0;
end


%% SpotCoord.yRed - SpotPhase.Phase3_*

coord = SpotCoord.yRed;

for phase = phases3_1to3_4
    paramRefGen(phase,coord).fun = SpotGnc.refConstantRate;
    paramRefGen(phase,coord).k1  = (-1 * startPhase - pi)*rRef;
    paramRefGen(phase,coord).k2  = omgRef * rRef;
end

for phase = [ SpotPhase.Phase4 SpotPhase.Phase5 SpotPhase.Phase6 ]
    paramRefGen(phase,coord).k1 = rRef * pi;
end


%% SpotCoord.*Black - default

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

for phase = allPhases
    paramRefGen(phase,SpotCoord.xBlack    ).k1 = init_states_BLACK(1);
    paramRefGen(phase,SpotCoord.yBlack    ).k1 = init_states_BLACK(2);
    paramRefGen(phase,SpotCoord.thetaBlack).k1 = init_states_BLACK(3) - pi/2;
end


%% SpotCoord.*Arm - default

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

for phase = allPhases

    paramRefGen(phase,SpotCoord.shoulderArm).k1 = pi/2;
    paramRefGen(phase,SpotCoord.elbowArm   ).k1 = pi/2;
    paramRefGen(phase,SpotCoord.wristArm   ).k1 = -1 * pi/2;

end

