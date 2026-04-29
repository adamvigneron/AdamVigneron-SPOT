%% predeclare for code generation
numPhase = length(meta.class.fromName('SpotPhase').EnumerationMemberList);
numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);

structCtrl.fun = SpotGnc.ctrlNone;
structCtrl.k1  = 0;
structCtrl.k2  = 0;
structCtrl.k3  = 0;
structCtrl.k4  = 0;

paramCtrl = repmat(structCtrl,numPhase,numCoord);


%% predeclare for simulink model

rRef   = 0.85;  % radius, metres
omgRef = 0.03490659;  % angular frequency, rad/s

myTime = 0:baseRate:tsim;
myData = -1 * omgRef^2 * rRef * ones(length(myTime),numCoord);

feedForward = timeseries(myData,myTime);


%% convenience variables

allPhases = enumeration('SpotPhase');
allPhases = allPhases(:).';  % converts to a row vector

phases2to5 = allPhases( (allPhases ~= SpotPhase.Phase0) & ...
                        (allPhases ~= SpotPhase.Phase1) & ...
                        (allPhases ~= SpotPhase.Phase6) );


%% SpotCoord.xRed - default

coord = SpotCoord.xRed;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd_vel;
    paramCtrl(phase,coord).k1  = K_RED(1,1) / mRED / 4;
    paramCtrl(phase,coord).k2  = K_RED(1,4) / mRED / sqrt(4);
end

for phase = [SpotPhase.Phase3_1, SpotPhase.Phase3_2, ...
             SpotPhase.Phase3_3, SpotPhase.Phase3_4]
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPdFwd_vel;
    paramCtrl(phase,coord).k4  = 1;
end


%% SpotCoord.yRed - default

coord = SpotCoord.yRed;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd_vel;
    paramCtrl(phase,coord).k1  = K_RED(2,2) / mRED / 4;
    paramCtrl(phase,coord).k2  = K_RED(2,5) / mRED / sqrt(4);
end


%% SpotCoord.thetaRed - default

coord = SpotCoord.thetaRed;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd_vel;
    paramCtrl(phase,coord).k1  = K_RED(3,3) / IRED;
    paramCtrl(phase,coord).k2  = K_RED(3,6) / IRED;
end


%% SpotCoord.xBlack - default

coord = SpotCoord.xBlack;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd_vel;
    paramCtrl(phase,coord).k1  = K_BLACK(1,1) / mBLACK;
    paramCtrl(phase,coord).k2  = K_BLACK(1,4) / mBLACK;
end


%% SpotCoord.yBlack - default

coord = SpotCoord.yBlack;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd_vel;
    paramCtrl(phase,coord).k1  = K_BLACK(2,2) / mBLACK;
    paramCtrl(phase,coord).k2  = K_BLACK(2,5) / mBLACK;
end


%% SpotCoord.thetaBlack - default

coord = SpotCoord.thetaBlack;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd_vel;
    paramCtrl(phase,coord).k1  = K_BLACK(3,3) / IBLACK;
    paramCtrl(phase,coord).k2  = K_BLACK(3,6) / IBLACK;
end

