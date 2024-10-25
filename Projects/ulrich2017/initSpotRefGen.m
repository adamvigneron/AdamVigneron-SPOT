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


%% reference orbit

rRef   = 0.85;  % radius, metres
omgRef = 0.03490659;  % angular frequency, rad/s


%% SpotPhase.Phase3_4 - SpotCoord.xRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.xRed;

paramRefGen(phase,coord).fun = SpotGnc.refCosine;

% ref = k1 * cos( k2 * t + k3 ) + k4
paramRefGen(phase,coord).k1  = rRef;
paramRefGen(phase,coord).k2  = omgRef;
paramRefGen(phase,coord).k3  = 0;
paramRefGen(phase,coord).k4  = init_states_RED(1) - rRef;


%% SpotPhase.Phase3_4 - SpotCoord.yRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.yRed;

paramRefGen(phase,coord).fun = SpotGnc.refSine;

% ref = k1 * sin( k2 * t + k3) + k4;
paramRefGen(phase,coord).k1  = rRef;
paramRefGen(phase,coord).k2  = omgRef;
paramRefGen(phase,coord).k3  = 0;
paramRefGen(phase,coord).k4  = init_states_RED(2);


%% SpotPhase.Phase3_4 - SpotCoord.thetaRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.thetaRed;

paramRefGen(phase,coord).fun = SpotGnc.refPolyWrap;

% ref = wrapToPi( k1 + k2 * t );
paramRefGen(phase,coord).k1  = pi;
paramRefGen(phase,coord).k2  = omgRef;
