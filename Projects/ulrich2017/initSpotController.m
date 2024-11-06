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
    paramCtrl(phase,coord).k1  = Kp_xr;
    paramCtrl(phase,coord).k2  = Kd_xr;
    paramCtrl(phase,coord).k3  = baseRate;  % 2*baseRate in experiment
end


%% SpotCoord.xRed - SpotPhase.Phase3_4

phase = SpotPhase.Phase3_4;
coord = SpotCoord.xRed;

paramCtrl(phase,coord).fun = SpotGnc.ctrlPdFwd_vel;
% paramCtrl(phase,coord).k1 has already been set
% paramCtrl(phase,coord).k2 has already been set
% paramCtrl(phase,coord).k3 has already been set
% paramCtrl(phase,coord).k4 will be overwritten with beta values at runtime


%% SpotCoord.yRed - default

coord = SpotCoord.yRed;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = Kp_yr;
    paramCtrl(phase,coord).k2  = Kd_yr;
    paramCtrl(phase,coord).k3  = baseRate;  % 2*baseRate in experiment
end


%% SpotCoord.yRed - SpotPhase.Phase3_4

phase = SpotPhase.Phase3_4;
coord = SpotCoord.yRed;

paramCtrl(phase,coord).fun = SpotGnc.ctrlPdFwd;
% paramCtrl(phase,coord).k1 has already been set
% paramCtrl(phase,coord).k2 has already been set
% paramCtrl(phase,coord).k3 has already been set
% paramCtrl(phase,coord).k4 will be overwritten with beta values at runtime


%% SpotCoord.thetaRed - default

coord = SpotCoord.thetaRed;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = Kp_tr;
    paramCtrl(phase,coord).k2  = Kd_tr;
    paramCtrl(phase,coord).k3  = baseRate;  % 2*baseRate in experiment
end


%% SpotCoord.thetaRed - SpotPhase.Phase3_4

phase = SpotPhase.Phase3_4;
coord = SpotCoord.thetaRed;

paramCtrl(phase,coord).fun = SpotGnc.ctrlPdFwd;
% paramCtrl(phase,coord).k1 has already been set
% paramCtrl(phase,coord).k2 has already been set
% paramCtrl(phase,coord).k3 has already been set
% paramCtrl(phase,coord).k4 will be overwritten with beta values at runtime


%% SpotCoord.xBlack - default

coord = SpotCoord.xBlack;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = Kp_xb;
    paramCtrl(phase,coord).k2  = Kd_xb;
    paramCtrl(phase,coord).k3  = baseRate;  % 2*baseRate in experiment
end


%% SpotCoord.yBlack - default

coord = SpotCoord.yBlack;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = Kp_yb;
    paramCtrl(phase,coord).k2  = Kd_yb;
    paramCtrl(phase,coord).k3  = baseRate;  % 2*baseRate in experiment
end


%% SpotCoord.thetaBlack - default

coord = SpotCoord.thetaBlack;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = Kp_tb;
    paramCtrl(phase,coord).k2  = Kd_tb;
    paramCtrl(phase,coord).k3  = baseRate;  % 2*baseRate in experiment
end


%% SpotCoord.xBlue - default

coord = SpotCoord.xBlue;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = Kp_xblue;
    paramCtrl(phase,coord).k2  = Kd_xblue;
    paramCtrl(phase,coord).k3  = baseRate;  % 2*baseRate in experiment
end


%% SpotCoord.yBlue - default

coord = SpotCoord.yBlue;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = Kp_yblue;
    paramCtrl(phase,coord).k2  = Kd_yblue;
    paramCtrl(phase,coord).k3  = baseRate;  % 2*baseRate in experiment
end


%% SpotCoord.thetaBlue - default

coord = SpotCoord.thetaBlue;

for phase = phases2to5
    paramCtrl(phase,coord).fun = SpotGnc.ctrlPd;
    paramCtrl(phase,coord).k1  = Kp_tblue;
    paramCtrl(phase,coord).k2  = Kd_tblue;
    paramCtrl(phase,coord).k3  = baseRate;  % 2*baseRate in experiment
end


%% SpotCoord.shoulderArm - default

% for now, this stays at SpotGnc.ctrlNone


%% SpotCoord.elbowArm - default

% for now, this stays at SpotGnc.ctrlNone


%% SpotCoord.wristArm - default

% for now, this stays at SpotGnc.ctrlNone

