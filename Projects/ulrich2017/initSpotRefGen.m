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

phases3_1to3_4 = [SpotPhase.Phase3_1, SpotPhase.Phase3_2, ...
                  SpotPhase.Phase3_3, SpotPhase.Phase3_4];


%% reference orbit

rRef   = 0.85;  % radius, metres
omgRef = 0.03490659;  % angular frequency, rad/s

% % aas_sfm_2025
% omgRef = 2 * omgRef;  % doubling to reduce chance of sticking on table

% we define a phase offset corresponding to the orbit start time
startPhase = omgRef * Phase2_End;


%% SpotPhase.Phase2 - Platforms

phase = SpotPhase.Phase2;

% paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant

% % aas_sfm_2025
% % update the drop/initial state of RED and BLACK
% drop_states_RED(2) = 1.9;
% init_states_RED(2) = 1.9;
% init_states_RED(3) = init_states_RED(3) + pi/2;
% init_states_BLACK(1) = init_states_RED(1) - rRef;
% init_states_BLACK(2) = 0.9;

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

    paramRefGen(phase,coord).fun = SpotGnc.refCosine;

    % ref = k1 * cos( k2 * t + k3 ) + k4
    paramRefGen(phase,coord).k1  = rRef;
    paramRefGen(phase,coord).k2  = omgRef;
    paramRefGen(phase,coord).k3  = -1 * startPhase;
    paramRefGen(phase,coord).k4  = init_states_RED(1) - rRef;

end


%% SpotPhase.Phase3 - SpotCoord.yRed

coord = SpotCoord.yRed;

for phase = phases3_1to3_4

    paramRefGen(phase,coord).fun = SpotGnc.refSine;

    % ref = k1 * sin( k2 * t + k3) + k4;
    paramRefGen(phase,coord).k1  = rRef;
    paramRefGen(phase,coord).k2  = omgRef;
    paramRefGen(phase,coord).k3  = -1 * startPhase;
    paramRefGen(phase,coord).k4  = init_states_RED(2);

    % % aas_sfm_2025
    % paramRefGen(phase,coord).k1  = paramRefGen(phase,coord).k1 * 0.2;  % reduce to stay in airflow
    % paramRefGen(phase,coord).k2  = paramRefGen(phase,coord).k2 * 2;    % double for figure-8
    % paramRefGen(phase,coord).k3  = paramRefGen(phase,coord).k3 * 2;    % same comment

end


%% SpotPhase.Phase3 - SpotCoord.thetaRed

coord = SpotCoord.thetaRed;

for phase = phases3_1to3_4
    
    paramRefGen(phase,coord).fun = SpotGnc.refPolyWrap;

    % ref = wrapToPi( k1 + k2 * t );
    paramRefGen(phase,coord).k1  = pi - startPhase;
    paramRefGen(phase,coord).k2  = omgRef;

    % % aas_sfm_2025
    % paramRefGen(phase,coord).fun = SpotGnc.refConstant;
    % paramRefGen(phase,coord).k1 = init_states_RED(3);

end


%% SpotPhase.Phase3 - SpotCoord.xBlack | SpotCoord.yBlack | spotCoord.thetaBlack

for phase = phases3_1to3_4

    % paramRefGen(phase,coord).fun has already been set to SpotGnc.refConstant
    paramRefGen(phase,SpotCoord.xBlack).k1 = init_states_BLACK(1);
    paramRefGen(phase,SpotCoord.yBlack).k1 = init_states_BLACK(2);
    
    % ref = wrapToPi( k1 + k2 * t );
    paramRefGen(phase,SpotCoord.thetaBlack).fun = SpotGnc.refPolyWrap;
    paramRefGen(phase,SpotCoord.thetaBlack).k1  = -1 * startPhase;
    paramRefGen(phase,SpotCoord.thetaBlack).k2  = omgRef;

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
    paramRefGen(phase,SpotCoord.wristArm   ).k1 = 0;

end

