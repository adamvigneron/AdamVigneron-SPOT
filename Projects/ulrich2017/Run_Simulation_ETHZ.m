%% INTRO

% A.P. Schoellig and R. D'Andrea
% "Optimization-​Based Iterative Learning Control for Trajectory Tracking"
% European Control Conference 2009, pp. 1505-​1510
% https://doi.org/10.23919/ECC.2009.7074619


%% SETUP

% change default figure style
set(groot,'DefaultFigureWindowStyle','docked');
set(groot,'DefaultLineLineWidth', 2);
set(groot,'DefaultAxesFontSize', 18);
set(groot,'defaultAxesXGrid','on');
set(groot,'defaultAxesYGrid','on');

% run the SPOT intialization script
Run_Initializer;

% get a handle for the open app via its containing figure
myFig = findall(groot,'Name','SPOT 4.0 GUI');
myApp = myFig.RunningAppInstance;

% select the first available simulink model
myApp.AvailableDiagramsDropDown.Value = myApp.AvailableDiagramsDropDown.Items{2};
myApp.public_AvailableDiagramsDropDownValueChanged([]);  % [] is an empty event

% activate all three platforms
myApp.REDCheckBox.Value = 1;
myApp.BLUECheckBox.Value = 1;
myApp.BLACKCheckBox.Value = 1;
myApp.public_REDCheckBoxValueChanged([]);
myApp.public_BLUECheckBoxValueChanged([]);
myApp.public_BLACKCheckBoxValueChanged([]);


%% CONFIGURATION

% we run the learning algorithm ten times slower than the model
sampleFactor = 10;
learnRate    = sampleFactor * baseRate;  % seconds

% determine timeseries indices in Phase 3 for the model and the learning
idxPh2_End = round(Phase2_End / baseRate) + 2;
idxPh3_End = round(Phase3_End / baseRate) + 2;
idxPh3     = idxPh2_End:idxPh3_End;
idxPh3_ILC = idxPh3(1:sampleFactor:end);


%% PLANT MODEL

% double integrator, continuous time
A = [0 1; 0 0];
B = [0; 1];
C = [1 0];
D = 0;

% double integrator, discrete time (zoh)
Ad = [1 learnRate; 0 1];
Bd = [0.5*learnRate^2; learnRate];
Cd = C;
Dd = D;

% dimensions
xDim   = size(B,1);
uDim   = size(B,2);
nSteps = round(Phase3_Duration/learnRate);


%% ETHZ MATRICES

% state equation, x = F*u + d

% precalculation
F_store        = zeros( xDim, uDim, nSteps);
F_store(:,:,2) = Bd;

d_store        = zeros( xDim,    1, nSteps);
d_store(:,:,1) = [drop_states_RED(1); 0];
d_store(:,:,2) = Ad * d_store(:,:,1);

for q = 3:nSteps+1
    F_store(:,:,q) = Ad * F_store(:,:,q-1);
    d_store(:,:,q) = Ad * d_store(:,:,q-1);
end
    
% F matrix
% d vector
F = zeros( (nSteps+1)*xDim, (nSteps+1)*uDim );
d = zeros( (nSteps+1)*xDim, 1 );

for l = 1:(nSteps+1)
    rowIdx = (1:xDim) + (l-1)*xDim;

    for m = 1:l
        colIdx = (1:uDim) + (m-1)*uDim;
        F(rowIdx,colIdx) = F_store(:,:,l-m+1);
    end

    d(rowIdx) = d_store(:,:,l);
end

% output equation, y = G*x + H*u

% G matrix
GCell = repmat({Cd},1,nSteps+1);
G     = blkdiag(GCell{:});

% H matrix
HCell = repmat({Dd},1,nSteps+1);
H     = blkdiag(HCell{:});


%% SIMULATION 1: WITHOUT COMPENSATION

% we choose to apply a disturbance in Phase3 on xRed
disturbFT.Data(idxPh3,SpotCoord.xRed) = 100e-3;  % force, newtons

% start the simulation
myEvent.Value = 'suppressHardwareWarning';
myApp.public_StartSimulationButtonPushed([]);

% extract the relevant data
tVec  = dataClass.Time_s(idxPh3_ILC);
rVec  = dataClass.RED_REFx_m(idxPh3_ILC);
uVec1 = dataClass.RED_Fx_Sat_N(idxPh3_ILC) / mRED;
yVec1 = dataClass.RED_Px_m(idxPh3_ILC);


%% COMMAND UPDATE

% nominal plant and controller model
tf_G = ss(A,B,C,D);
tf_K = pid(Kp_xr,0,Kd_xr,baseRate);

% closed-loop transfer functions
tf_UR = feedback(tf_K,tf_G);
tf_YR = feedback(tf_K*tf_G,1);

% normalize closed-loop reference
rVec0 = rVec - drop_states_RED(1);

% map closed-loop reference onto control and output
uVec0 = lsim(tf_UR,rVec0,tVec);
yVec0 = lsim(tf_YR,rVec0,tVec) + drop_states_RED(1);

% [ETHZ] calculate departure from nominal trajectory
uTilde = uVec1 - uVec0;
yTilde = yVec1 - yVec0;

% [ETHZ] calculate modelling error estimate and updated input 
dHat = G\yTilde - F*uTilde - G\H*uTilde;
uNew = -F\dHat;

% resample the updated input as a feed-forward force in Phase3;
% interpolation and extrapolation using the 'previous' mathod
fNew = interp1(tVec, mRED*uNew, feedForward.Time(idxPh3), 'previous', 'extrap');


%% SIMULATION 2: WITH COMPENSATION

% update the feed-forward signal
paramCtrl(SpotPhase.Phase3_4,SpotCoord.xRed).fun = SpotGnc.ctrlPdFwd;
paramCtrl(SpotPhase.Phase3_4,SpotCoord.xRed).k4  = 1;
feedForward.Data(idxPh3,SpotCoord.xRed)          = fNew;

% start the simulation
myEvent.Value = 'suppressHardwareWarning';
myApp.public_StartSimulationButtonPushed([]);

% extract the relevant data
uVec2 = dataClass.RED_Fx_Sat_N(idxPh3_ILC) / mRED;
yVec2 = dataClass.RED_Px_m(idxPh3_ILC);

uFbk2 = (dataClass.RED_Fx_Kp_N(idxPh3_ILC) + dataClass.RED_Fx_Kd_N(idxPh3_ILC) ) / mRED;
uFwd2 =  dataClass.RED_Fx_u0_N(idxPh3_ILC) / mRED;

%% SIMULATION 3: FURTHER IMPROVEMENTS?

% repeat the command update
uTilde = uVec2 - uVec0;
yTilde = yVec2 - yVec0;

dHat = G\yTilde - F*uTilde - G\H*uTilde;
uNew = -F\dHat;

fNew = interp1(tVec, mRED*uNew, feedForward.Time(idxPh3), 'previous', 'extrap');
feedForward.Data(idxPh3,SpotCoord.xRed) = fNew;

% start the simulation
myEvent.Value = 'suppressHardwareWarning';
myApp.public_StartSimulationButtonPushed([]);

% extract the relevant data
uVec3 = dataClass.RED_Fx_Sat_N(idxPh3_ILC) / mRED;
yVec3 = dataClass.RED_Px_m(idxPh3_ILC);


%% PLOT

% output signal
figure('name','RED_Px_m (value)')
plot(tVec,[yVec0 yVec1 yVec2 yVec3]);
legend('nominal','1: feedback', '2: feedback + feedfwd', ...
       '3: further improvement', 'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_Px\_m');
title('output signal');

% output error
figure('name','RED_Px_m (error)')
plot(tVec,[yVec0-rVec yVec1-rVec yVec2-rVec yVec3-rVec]);
legend('nominal','1: feedback', '2: feedback + feedfwd', ...
       '3: further improvement', 'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_Px\_m');
title('output error');

% control signal
figure('name','RED_Fx_Sat_N')
plot(tVec,mRED * [uVec0 uVec1 uVec2 uFbk2 uFwd2 uVec3]);
hold on;
plot(disturbFT.Time(idxPh3,SpotCoord.xRed),-1 * disturbFT.Data(idxPh3,SpotCoord.xRed),'k--')
legend('nominal','1: feedback', '2: feedback + feedfwd', ...
       '2: feedback', '2: feedfwd', '3: further improvement', ...
       [char(8211) '1 \times disturbance'], 'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_Fx\_Sat\_N');
title('control signal');
ylim([-0.2 0.1]);


% %% SAVE
% 
% % save the simulation data
% myEvent.Value = 'suppressWaitfor';
% myApp.public_SaveSimulationDataButtonPushed(myEvent);
% 
% 
% %% SHUTDOWN
% 
% % reset figure style
% reset(groot);
% 
% % close the simulink model; close the figure (and app); clear the workspace
% bdclose;
% close(myFig);
% clear;

