%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);

structPhaseMgmt.phaseEnd = 0;

paramPhaseMgmt = repmat(structPhaseMgmt,numPhase);


%% use phase durations to define phase ends 

paramPhaseMgmt(SpotPhase.Phase0).phaseEnd = 10 ;
paramPhaseMgmt(SpotPhase.Phase1).phaseEnd =  5 + paramPhaseMgmt(SpotPhase.Phase0).phaseEnd;
paramPhaseMgmt(SpotPhase.Phase2).phaseEnd = 40 + paramPhaseMgmt(SpotPhase.Phase1).phaseEnd;

paramPhaseMgmt(SpotPhase.Phase3_1).phaseEnd =  10 + paramPhaseMgmt(SpotPhase.Phase2).phaseEnd;
paramPhaseMgmt(SpotPhase.Phase3_2).phaseEnd =  10 + paramPhaseMgmt(SpotPhase.Phase3_1).phaseEnd;
paramPhaseMgmt(SpotPhase.Phase3_3).phaseEnd =  30 + paramPhaseMgmt(SpotPhase.Phase3_2).phaseEnd;
paramPhaseMgmt(SpotPhase.Phase3_4).phaseEnd = 120 + paramPhaseMgmt(SpotPhase.Phase3_3).phaseEnd;

paramPhaseMgmt(SpotPhase.Phase4).phaseEnd = 30 + paramPhaseMgmt(SpotPhase.Phase3_4).phaseEnd;
paramPhaseMgmt(SpotPhase.Phase5).phaseEnd = 20 + paramPhaseMgmt(SpotPhase.Phase4).phaseEnd;

