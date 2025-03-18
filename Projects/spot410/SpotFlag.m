function [puckState,armState] = SpotFlag(phase, paramFlag)

    %% assign output variables
    
    armState  = paramFlag(phase).armState;
    puckState = paramFlag(phase).puckState;

