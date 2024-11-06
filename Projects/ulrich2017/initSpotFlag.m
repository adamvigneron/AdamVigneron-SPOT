%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);

structFlag.rwState   = 0;
structFlag.armState  = 0;
structFlag.puckState = 0;

paramFlag = repmat(structFlag,1,numPhase);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector

phases1to5 = allPhases( (allPhases ~= SpotPhase.Phase0) & ...
                        (allPhases ~= SpotPhase.Phase6) );

phases2to5 = allPhases( (allPhases ~= SpotPhase.Phase0) & ...
                        (allPhases ~= SpotPhase.Phase1) & ...
                        (allPhases ~= SpotPhase.Phase6) );

%% paramFlag.rwState

% paramFlag.rwState is already set to 0 for all phases


%% paramFlag.armState

for phase = phases2to5
    paramFlag(phase).armState = 1;
end


%% paramFlag.puckState

for phase = phases1to5
    paramFlag(phase).puckState = 1;
end

