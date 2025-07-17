function [phase] = SpotPhaseMgmt(t, paramPhaseMgmt)

    % assign output phase as a function of input time
    if t < paramPhaseMgmt(SpotPhase.Phase0).phaseEnd
        phase = SpotPhase.Phase0;
    
    elseif t < paramPhaseMgmt(SpotPhase.Phase1).phaseEnd
        phase = SpotPhase.Phase1;
    
    elseif t < paramPhaseMgmt(SpotPhase.Phase2).phaseEnd
        phase = SpotPhase.Phase2; 
    
    elseif t < paramPhaseMgmt(SpotPhase.Phase3_1).phaseEnd
        phase = SpotPhase.Phase3_1;
    
    elseif t < paramPhaseMgmt(SpotPhase.Phase3_2).phaseEnd
        phase = SpotPhase.Phase3_2;
    
    elseif t < paramPhaseMgmt(SpotPhase.Phase3_3).phaseEnd
        phase = SpotPhase.Phase3_3;
    
    elseif t < paramPhaseMgmt(SpotPhase.Phase3_4).phaseEnd
        phase = SpotPhase.Phase3_4;
    
    elseif t < paramPhaseMgmt(SpotPhase.Phase4).phaseEnd
        phase = SpotPhase.Phase4;
    
    elseif t < paramPhaseMgmt(SpotPhase.Phase5).phaseEnd
        phase = SpotPhase.Phase5;
    
    else
        phase = SpotPhase.Phase6;
    
    end  % if

end  % function

