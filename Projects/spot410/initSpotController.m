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

myTime = 0:baseRate:tsim;
myData = zeros(length(myTime),numCoord);

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
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  =      4  * K_RED(1,1) / mRED;
    paramCtrl(phase,coord).k2  = sqrt(4) * K_RED(1,4) / mRED;
    paramCtrl(phase,coord).k3  = baseRate;
end


%% SpotCoord.yRed - default

coord = SpotCoord.yRed;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  =      4  * K_RED(2,2) / mRED;
    paramCtrl(phase,coord).k2  = sqrt(4) * K_RED(2,5) / mRED;
    paramCtrl(phase,coord).k3  = baseRate;
end


%% SpotCoord.thetaRed - default

coord = SpotCoord.thetaRed;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = K_RED(3,3) / IRED;
    paramCtrl(phase,coord).k2  = K_RED(3,6) / IRED;
    paramCtrl(phase,coord).k3  = baseRate;
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

