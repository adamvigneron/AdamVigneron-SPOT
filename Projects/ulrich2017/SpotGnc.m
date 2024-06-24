classdef SpotGnc < Simulink.IntEnumType
    enumeration
        refConstant(101)
        refCosine(102)
        refSine(103)
        refPolyWrap(104)
%        
        ctrlNone(201)
        ctrlPd(202)
        ctrlPdFwd(203)
        ctrlPd_vel(204)
        ctrlPdFwd_vel(205)
%
        errMinus(301)
        errMinusWrap(302)
        errHough(303)
%
        estNone(401)
        estVelBias(402)
    end
end

