%% predeclare for code generation
spotPhases = {meta.class.fromName('SpotPhase').EnumerationMemberList.Name};
spotCoords = {meta.class.fromName('SpotCoord').EnumerationMemberList.Name};

numPhase = length(spotPhases);
numCoord = length(spotCoords);

structCtrl.fun = SpotKey.ctrlNone;
structCtrl.k1  = 0;
structCtrl.k2  = 0;
structCtrl.k3  = 0;
structCtrl.k4  = 0;

paramCtrl = repmat(structCtrl,numPhase,numCoord);


%% predeclare for simulink model

myTime = 0:baseRate:tsim;
myData = zeros(length(myTime),numCoord);

feedForward = timeseries(myData,myTime);


%% SpotPhase.Phase3_4 - SpotCoord.xRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.xRed;

paramCtrl(phase,coord).fun = SpotKey.ctrlPdFwd;
paramCtrl(phase,coord).k1  = Kp_xr;
paramCtrl(phase,coord).k2  = Kd_xr;
paramCtrl(phase,coord).k3  = baseRate;
paramCtrl(phase,coord).k4  = 0;  % initial beta value


%% SpotPhase.Phase3_4 - SpotCoord.yRed

phase = SpotPhase.Phase3_4;
coord = SpotCoord.yRed;

paramCtrl(phase,coord).fun = SpotKey.ctrlPdFwd;
paramCtrl(phase,coord).k1  = Kp_yr;
paramCtrl(phase,coord).k2  = Kd_yr;
paramCtrl(phase,coord).k3  = baseRate;
paramCtrl(phase,coord).k4  = 0;  % initial beta value

