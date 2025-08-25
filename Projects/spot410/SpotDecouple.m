function [FTcmd] = SpotDecouple(phase, thetaRed, cmd, paramDcpl)

    %% initialization of output variables
    
    coords   = enumeration( SpotCoord(1) );
    numCoord = length(coords);

    FTcmd = zeros(numCoord,1);
    
    
    %% loop over all measurements

    for k = 1:numCoord
        
        coord = coords(k);

        %% select a decoupling method
        myFun = paramDcpl(phase,coord).fun;
        
        switch myFun
    
            case { SpotGnc.dcplSingleAxis , SpotGnc.dcplSingleAxisInvert }

                k1 = paramDcpl(phase,coord).k1;  % mass / inertia 

                FTcmd(coord) = k1 * cmd(coord);

                if myFun == SpotGnc.dcplSingleAxisInvert
                    FTcmd(coord) = -1 * FTcmd(coord);
                end


            case SpotGnc.dcplArmSetpoint

                FTcmd(coord) = cmd(coord);


            case SpotGnc.dcplRedBodyForce

                switch coord

                    case SpotCoord.yRed
                        % do nothing

                    case SpotCoord.xRed

                        k1 = paramDcpl(phase,coord).k1;  % mass of RED

                        C_BI = [ cos(thetaRed) -sin(thetaRed)
                                 sin(thetaRed)  cos(thetaRed) ];

                        FT_B = k1 * [ cmd(SpotCoord.xRed); 
                                      cmd(SpotCoord.yRed) ];

                        FT_I = C_BI * FT_B;

                        FTcmd(SpotCoord.xRed) = -1 * FT_I(1);
                        FTcmd(SpotCoord.yRed) = -1 * FT_I(2);

                    otherwise
                        error('SpotDecouple.m:\n  function dcplSingleAxisInvertRotate only defined for xRed and yRed');

                end  % switch coord


            otherwise
                error('SpotDecouple.m:\n  function SpotGnc(%d) not defined for SpotPhase(%d) and SpotCoord(%d).\n\n', int32(myFun), int32(phase), int32(coord))
    
        end % switch myFun

    end % loop coords

end % function

