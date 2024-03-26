%% configuration switches

includeSpinup = 1;


%% additional calculations

% as an initial conservative design, reserve 90% of the authority 
% and split the remaining 10% between spin-up and centripetal

% determine translational authority allocated for spin-up and centripetal
alloc    = 0.1;
traAuth  = min(F_red_X_nominal,F_red_Y_nominal) / mRED;
accAvail = alloc * traAuth;

% reference orbit 
rRef   = 0.85;  % radius, metres
omgRef = 0.03490659;  % angular frequency, rad/s

% calculate the centripetal and spin-up accelerations
accCent = omgRef^2 * rRef;
accSpin = accAvail - accCent;

% finally, calculate the total spin-up time
spinupTime = omgRef * rRef / accSpin;


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

%% SpotPhase.Phase3_4 - SpotCoord.xRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.xRed;

if includeSpinup
    paramRefGen(phase,coord).fun = SpotGnc.refCosineSpinup;
    paramRefGen(phase,coord).k5  = spinupTime;
else
    paramRefGen(phase,coord).fun = SpotGnc.refCosine;
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
    paramRefGen(phase,coord).fun = SpotGnc.refSineSpinup;
    paramRefGen(phase,coord).k5  = spinupTime;
else
    paramRefGen(phase,coord).fun = SpotGnc.refSine;
end

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
