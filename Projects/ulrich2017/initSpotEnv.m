%% predeclare for simulink model

numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);


%% plant input noise / disturbance

noiseFT.mean = zeros(numCoord,1);
noiseFT.seed = zeros(numCoord,1);
noiseFT.var  = zeros(numCoord,1);

myTime = 0:baseRate:tsim;
myData = zeros(length(myTime),numCoord);
disturbFT = timeseries(myData,myTime);

