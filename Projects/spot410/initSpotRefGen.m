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


%% SpotPhase.Phase2 - Platforms

phase = SpotPhase.Phase2;

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

paramRefGen(phase,SpotCoord.xRed    ).k1 = init_states_RED(1);
paramRefGen(phase,SpotCoord.yRed    ).k1 = init_states_RED(2);
paramRefGen(phase,SpotCoord.thetaRed).k1 = init_states_RED(3);

paramRefGen(phase,SpotCoord.xBlack    ).k1 = init_states_BLACK(1);
paramRefGen(phase,SpotCoord.yBlack    ).k1 = init_states_BLACK(2);
paramRefGen(phase,SpotCoord.thetaBlack).k1 = init_states_BLACK(3);

paramRefGen(phase,SpotCoord.xBlue    ).k1 = init_states_BLUE(1);
paramRefGen(phase,SpotCoord.yBlue    ).k1 = init_states_BLUE(2);
paramRefGen(phase,SpotCoord.thetaBlue).k1 = init_states_BLUE(3);


%% SpotPhase.Phase3 - SpotCoord.xRed

coord = SpotCoord.xRed;

for phase = phases3_1to3_4

    paramRefGen(phase,coord).fun = SpotGnc.refCircularInspection;
    % paramRefGen(phase,coord).sensor has already been set for SpotCoord.xRed

    % ref(SpotCoord.xRed)     = k1 * cos( k2 * t + k3 + thetaCorr ) + xCtr;
    % ref(SpotCoord.yRed)     = k1 * sin( k2 * t + k3 + thetaCorr ) + yCtr;
    % ref(SpotCoord.thetaRed) = wrapToPi( k2 * t + k4 );
    paramRefGen(phase,coord).k1  = rRef;
    paramRefGen(phase,coord).k2  = omgRef;
    paramRefGen(phase,coord).k3  = -1 * startPhase;
    paramRefGen(phase,coord).k4  = pi - startPhase;

end


%% SpotPhase.Phase3 - SpotCoord.yRed

coord = SpotCoord.yRed;

for phase = phases3_1to3_4

    paramRefGen(phase,coord).fun = SpotGnc.refCircularInspection;

    % paramRefGen(phase,coord).* have already been set for SpotCoord.yRed

end


%% SpotPhase.Phase3 - SpotCoord.thetaRed

coord = SpotCoord.thetaRed;

for phase = phases3_1to3_4
    
    paramRefGen(phase,coord).fun = SpotGnc.refCircularInspection;

    % paramRefGen(phase,coord).* have already been set for SpotCoord.thetaRed
    
end


%% SpotPhase.Phase3 - SpotCoord.xBlack | SpotCoord.yBlack | spotCoord.thetaBlack

for phase = phases3_1to3_4

    % paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant
    paramRefGen(phase,SpotCoord.xBlack).k1     = init_states_BLACK(1);
    paramRefGen(phase,SpotCoord.yBlack).k1     = init_states_BLACK(2);
    paramRefGen(phase,SpotCoord.thetaBlack).k1 = init_states_BLACK(3);

end


%% SpotPhase.Phase3 - SpotCoord.xBlue

coord = SpotCoord.xBlue;

for phase = phases3_1to3_4

    paramRefGen(phase,coord).fun = SpotGnc.refCosine;

    % ref = k1 * cos( k2 * t + k3 ) + k4
    paramRefGen(phase,coord).k1  = rRef;
    paramRefGen(phase,coord).k2  = omgRef;
    paramRefGen(phase,coord).k3  = pi - startPhase;
    paramRefGen(phase,coord).k4  = 0.5 * xLength;

end


%% SpotPhase.Phase3 - SpotCoord.yBlue

coord = SpotCoord.yBlue;

for phase = phases3_1to3_4

    paramRefGen(phase,coord).fun = SpotGnc.refSine;

    % ref = k1 * sin( k2 * t + k3) + k4;
    paramRefGen(phase,coord).k1  = rRef;
    paramRefGen(phase,coord).k2  = omgRef;
    paramRefGen(phase,coord).k3  = pi - startPhase;
    paramRefGen(phase,coord).k4  = 0.5 * yLength;

end


%% SpotPhase.Phase3 - SpotCoord.thetaBlue

coord = SpotCoord.thetaBlue;

for phase = phases3_1to3_4

    paramRefGen(phase,coord).fun = SpotGnc.refPolyWrap;

    % ref = wrapToPi( k1 + k2 * t );
    paramRefGen(phase,coord).k1  = -1 * startPhase;
    paramRefGen(phase,coord).k2  = omgRef;

end


%% SpotPhase.Phase4 | SpotPhase.Phase5 - Platforms

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

for phase = [SpotPhase.Phase4, SpotPhase.Phase5]

    paramRefGen(phase,SpotCoord.xRed    ).k1 = home_states_RED(1);
    paramRefGen(phase,SpotCoord.yRed    ).k1 = home_states_RED(2);
    paramRefGen(phase,SpotCoord.thetaRed).k1 = home_states_RED(3);
    
    paramRefGen(phase,SpotCoord.xBlack    ).k1 = home_states_BLACK(1);
    paramRefGen(phase,SpotCoord.yBlack    ).k1 = home_states_BLACK(2);
    paramRefGen(phase,SpotCoord.thetaBlack).k1 = home_states_BLACK(3);
    
    paramRefGen(phase,SpotCoord.xBlue    ).k1 = home_states_BLUE(1);
    paramRefGen(phase,SpotCoord.yBlue    ).k1 = home_states_BLUE(2);
    paramRefGen(phase,SpotCoord.thetaBlue).k1 = home_states_BLUE(3);

end


%% SpotCoord.shoulderArm | SpotCoord.elbowArm | SpotCoord.wristArm - default

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

for phase = allPhases

    paramRefGen(phase,SpotCoord.shoulderArm).k1 = pi/2;
    paramRefGen(phase,SpotCoord.elbowArm   ).k1 = pi/2;
    paramRefGen(phase,SpotCoord.wristArm   ).k1 = -1 * pi/2;

end

