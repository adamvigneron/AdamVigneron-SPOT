function [phase] = SpotPhaseMgmt(t, paramPhaseMgmt)

    % assign output phase as a function of input time
    if t < paramPhaseMgmt.Phase0_End
        phase = SpotPhase.Phase0;
    
    elseif t < paramPhaseMgmt.Phase1_End
        phase = SpotPhase.Phase1;
    
    elseif t < paramPhaseMgmt.Phase2_End
        phase = SpotPhase.Phase2; 
    
    elseif t < paramPhaseMgmt.Phase3_SubPhase1_End
        phase = SpotPhase.Phase3_1;
    
    elseif t < paramPhaseMgmt.Phase3_SubPhase2_End
        phase = SpotPhase.Phase3_2;
    
    elseif t < paramPhaseMgmt.Phase3_SubPhase3_End
        phase = SpotPhase.Phase3_3;
    
    elseif t < paramPhaseMgmt.Phase3_SubPhase4_End
        phase = SpotPhase.Phase3_4;
    
    elseif t < paramPhaseMgmt.Phase4_End
        phase = SpotPhase.Phase4;
    
    elseif t < paramPhaseMgmt.Phase5_End
        phase = SpotPhase.Phase5;
    
    else
        phase = SpotPhase.Phase6;
    
    end  % if

end  % function

