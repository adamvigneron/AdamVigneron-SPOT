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
    paramCtrl(phase,coord).k1  = -1 *      2  * K_RED(1,1);
    paramCtrl(phase,coord).k2  = -1 * sqrt(2) * K_RED(1,4);
    paramCtrl(phase,coord).k3  = baseRate;
end


%% SpotCoord.yRed - default

coord = SpotCoord.yRed;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = -1 *      2  * K_RED(2,2);
    paramCtrl(phase,coord).k2  = -1 * sqrt(2) * K_RED(2,5);
    paramCtrl(phase,coord).k3  = baseRate;
end


%% SpotCoord.thetaRed - default

coord = SpotCoord.thetaRed;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = -1 * K_RED(3,3);
    paramCtrl(phase,coord).k2  = -1 * K_RED(3,6);
    paramCtrl(phase,coord).k3  = baseRate;
end


%% SpotCoord.thetaRed - SpotPhase.Phase3_*

coord = SpotCoord.thetaRed;

for phase = phases3_1to3_4
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd_vel;
    paramCtrl(phase,coord).k1  = 0;
    paramCtrl(phase,coord).k2  = K_RED(3,6);
end


%% SpotCoord.xBlack - default

% paramCtrl(phase,coord).fun is already set to ctrlNone


%% SpotCoord.yBlack - default

% paramCtrl(phase,coord).fun is already set to ctrlNone


%% SpotCoord.thetaBlack - default

% paramCtrl(phase,coord).fun is already set to ctrlNone


%% SpotCoord.xBlue - default

% paramCtrl(phase,coord).fun is already set to ctrlNone


%% SpotCoord.yBlue - default

% paramCtrl(phase,coord).fun is already set to ctrlNone


%% SpotCoord.thetaBlue - default

% paramCtrl(phase,coord).fun is already set to ctrlNone


%% SpotCoord.shoulderArm - default

% paramCtrl(phase,coord).fun is already set to ctrlNone


%% SpotCoord.elbowArm - default

% paramCtrl(phase,coord).fun is already set to ctrlNone


%% SpotCoord.wristArm - default

% paramCtrl(phase,coord).fun is already set to ctrlNone

