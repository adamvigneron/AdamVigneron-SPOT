%% predeclare for code generation

numPhase = length( enumeration( SpotPhase(1) ) );
numCoord = length( enumeration( SpotCoord(1) ) );

paramRefGen = repmat(struct, numPhase, numCoord);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector

allCoords = enumeration('SpotCoord');
allCoords = allCoords(:).';  % converts to a row vector

phases3_1to3_4 = [SpotPhase.Phase3_1, SpotPhase.Phase3_2, ...
                  SpotPhase.Phase3_3, SpotPhase.Phase3_4];


%% by default, every coordinate uses its Phasespace measurement

for phase = allPhases
    for coord = allCoords
        paramRefGen(phase,coord).fun    = SpotGnc.refConstant;
        paramRefGen(phase,coord).sensor = SpotSensor(coord.real);
        paramRefGen(phase,coord).k1     = 0;
        paramRefGen(phase,coord).k2     = 0;
        paramRefGen(phase,coord).k3     = 0;
        paramRefGen(phase,coord).k4     = 0;
        paramRefGen(phase,coord).k5     = 0;
    end
end


%% reference orbit

rRef   = 0.85;  % radius, metres
omgRef = 0.03490659;  % angular frequency, rad/s

% we define a phase offset corresponding to the orbit start time
startPhase = omgRef * Phase2_End;


%% SpotCoord.xRed | spotCoord.yRed - default

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

for phase = allPhases
    paramRefGen(phase,SpotCoord.xRed).k1 = rRef;
    paramRefGen(phase,SpotCoord.yRed).k1 = 0;
end


%% SpotCoord.thetaRed - default

coord = SpotCoord.thetaRed;

for phase = allPhases
    paramRefGen(phase,coord).fun = SpotGnc.refConstant;
    paramRefGen(phase,coord).k1  = 0;
end


%% SpotCoord.thetaRed - SpotPhase.Phase3_*

coord = SpotCoord.thetaRed;

for phase = phases3_1to3_4
    paramRefGen(phase,coord).fun = SpotGnc.refConstantRate;
    paramRefGen(phase,coord).k1  = -1 * startPhase;
    paramRefGen(phase,coord).k2  = omgRef;
end


%% SpotCoord.*Black - default

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

for phase = allPhases
    paramRefGen(phase,SpotCoord.xBlack    ).k1 = init_states_BLACK(1);
    paramRefGen(phase,SpotCoord.yBlack    ).k1 = init_states_BLACK(2);
    paramRefGen(phase,SpotCoord.thetaBlack).k1 = init_states_BLACK(3);
end


%% SpotCoord.shoulderArm | SpotCoord.elbowArm | SpotCoord.wristArm - default

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

for phase = allPhases

    paramRefGen(phase,SpotCoord.shoulderArm).k1 = pi/2;
    paramRefGen(phase,SpotCoord.elbowArm   ).k1 = pi/2;
    paramRefGen(phase,SpotCoord.wristArm   ).k1 = -1 * pi/2;

end

