classdef SpotKey < Simulink.IntEnumType
    enumeration
        refConstant(101)
        refCosine(102)
        refSine(103)
        refCosineSpinup(104)
        refSineSpinup(105)
        refPolyWrap(106)
%        
        ctrlNone(201)
        ctrlPd(202)
        ctrlPdFwd(203)
    end
end

