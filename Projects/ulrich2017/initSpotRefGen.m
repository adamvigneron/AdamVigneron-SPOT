%% configuration switches

includeSpinup = 1;


%% additional calculations

% as an initial conservative design, reserve half of the authority 
% and split the remaining half between spin-up and centripetal

alloc = 0.1;

traAuth = min(F_red_X_nominal,F_red_Y_nominal) / mRED;
accAvail = alloc * traAuth;

rRef   = 0.85;
omgRef = 0.03490659;

accCent = omgRef^2 * rRef;
accSpin = accAvail - accCent;

spinupTime = omgRef * rRef / accSpin;


%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structRefGen.fun = SpotKey.refConstant;
structRefGen.k1  = 0;
structRefGen.k2  = 0;
structRefGen.k3  = 0;
structRefGen.k4  = 0;
structRefGen.k5  = 0;

paramRefGen = repmat(structRefGen,numPhase,numCoord);

%% SpotPhase.Phase3_4 - SpotCoord.xRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.xRed;

if includeSpinup
    paramRefGen(phase,coord).fun = SpotKey.refCosineSpinup;
    paramRefGen(phase,coord).k5  = spinupTime;
else
    paramRefGen(phase,coord).fun = SpotKey.refCosine;
end

% ref = k1 * cos( k2 * t + k3 ) + k4
paramRefGen(phase,coord).k1  = rRef;
paramRefGen(phase,coord).k2  = omgRef;
paramRefGen(phase,coord).k3  = 0;
paramRefGen(phase,coord).k4  = init_states_RED(1) - rRef;


%% SpotPhase.Phase3_4 - SpotCoord.yRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.yRed;

if includeSpinup
    paramRefGen(phase,coord).fun = SpotKey.refSineSpinup;
    paramRefGen(phase,coord).k5  = spinupTime;
else
    paramRefGen(phase,coord).fun = SpotKey.refSine;
end

% ref = k1 * sin( k2 * t + k3) + k4;
paramRefGen(phase,coord).k1  = rRef;
paramRefGen(phase,coord).k2  = omgRef;
paramRefGen(phase,coord).k3  = 0;
paramRefGen(phase,coord).k4  = init_states_RED(2);

