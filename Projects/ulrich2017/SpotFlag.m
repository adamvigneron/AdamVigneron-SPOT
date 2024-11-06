function [puckState,rwState,armState] = SpotFlag(phase, paramFlag)

    %% assign output variables
    
    rwState   = paramFlag(phase).rwState;
    armState  = paramFlag(phase).armState;
    puckState = paramFlag(phase).puckState;

