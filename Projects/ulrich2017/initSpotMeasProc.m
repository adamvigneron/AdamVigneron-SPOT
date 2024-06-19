%% predeclare for simulink model

numCoord = length(meta.class.fromName('SpotCoord').EnumerationMemberList);


%% measurement noise

noiseMeas.mean = zeros(numCoord,1);
noiseMeas.seed = zeros(numCoord,1);
noiseMeas.var  = zeros(numCoord,1);



