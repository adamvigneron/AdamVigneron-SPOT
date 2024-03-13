%% configuration switches

includeSpinup = 1;


%% additional calculations

spinupTime = 100;


%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structRefGen.fun = SpotRef.constant;
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
    paramRefGen(phase,coord).fun = SpotRef.cosineSpinup;
    paramRefGen(phase,coord).k5  = spinupTime;
else
    paramRefGen(phase,coord).fun = SpotRef.cosine;
end

% ref = k1 * cos( k2 * t + k3 ) + k4
paramRefGen(phase,coord).k1  = 0.85;
paramRefGen(phase,coord).k2  = 0.03490659;
paramRefGen(phase,coord).k3  = 0;
paramRefGen(phase,coord).k4  = xLength / 2;

% we can update the definition of k4 to ensure a bumpless transfer
paramRefGen(phase,coord).k4  = init_states_RED(1) - paramRefGen(phase,coord).k1;


%% SpotPhase.Phase3_4 - SpotCoord.yRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.yRed;

if includeSpinup
    paramRefGen(phase,coord).fun = SpotRef.sineSpinup;
    paramRefGen(phase,coord).k5  = spinupTime;
else
    paramRefGen(phase,coord).fun = SpotRef.sine;
end

% ref = k1 * sin( k2 * t + k3) + k4;
paramRefGen(phase,coord).k1  = 0.85;
paramRefGen(phase,coord).k2  = 0.03490659;
paramRefGen(phase,coord).k3  = 0;
paramRefGen(phase,coord).k4  = yLength / 2;

% we can update the definition of k4 to ensure a bumpless transfer
paramRefGen(phase,coord).k4  = init_states_RED(2);

