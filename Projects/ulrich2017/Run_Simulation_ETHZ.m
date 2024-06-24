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

% kalman filter
flag_kalman = 1;

% noise switches
flag_distFT    = 1;
flag_walkFT    = 1;
flag_noiseFT   = 1;
flag_noiseMeas = 1;

% noise and disturbance values
disturbFT_xRed      = 0.1;   % newton, absolute
walkFT_xRed_3sig    = 0.02;  % newton, three-sigma
noiseFT_xRed_3sig   = 0.02;  % newton, three-sigma
noiseMeas_xRed_3sig = 0.01;  % metre, three-sigma

% we run the learning algorithm ten times slower than the model
sampleFactor = 10;
learnRate    = sampleFactor * baseRate;  % seconds

% determine timeseries indices in Phase 3 for the model and the learning
idxPh2_End = round(Phase2_End / baseRate) + 2;
idxPh3_End = round(Phase3_End / baseRate) + 2;
idxPh3     = idxPh2_End:idxPh3_End;
idxPh3_ILC = idxPh3(1:sampleFactor:end);
idxPh3_ILC = idxPh3_ILC(1:end-1);
nSteps     = length(idxPh3_ILC);


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
yDim   = size(C,1);


%% NOISE CHARACTERISTICS

if flag_noiseFT
    noiseFT.var(SpotCoord.xRed) = ( noiseFT_xRed_3sig / 3 )^2; 
end

if flag_noiseMeas
    noiseMeas.var(SpotCoord.xRed) = ( noiseMeas_xRed_3sig / 3 )^2;  
end


%% ETHZ MATRICES

% state equation, x = F*u + d

% precalculation
F_store        = zeros( xDim, uDim, nSteps);
F_store(:,:,2) = Bd;

d_store        = zeros( xDim,    1, nSteps);
d_store(:,:,1) = [drop_states_RED(1); 0];
d_store(:,:,2) = Ad * d_store(:,:,1);

for q = 3:nSteps
    F_store(:,:,q) = Ad * F_store(:,:,q-1);
    d_store(:,:,q) = Ad * d_store(:,:,q-1);
end
    
% F matrix
% d vector
F = zeros( nSteps*xDim, nSteps*uDim );
d = zeros( nSteps*xDim, 1 );

for l = 1:nSteps
    rowIdx = (1:xDim) + (l-1)*xDim;

    for m = 1:l
        colIdx = (1:uDim) + (m-1)*uDim;
        F(rowIdx,colIdx) = F_store(:,:,l-m+1);
    end

    d(rowIdx) = d_store(:,:,l);
end

% output equation, y = G*x + H*u

% G matrix
GCell = repmat({Cd},1,nSteps);
G     = blkdiag(GCell{:});

% H matrix
HCell = repmat({Dd},1,nSteps);
H     = blkdiag(HCell{:});


%% SIMULATION 1: WITHOUT COMPENSATION

% we apply a disturbance in Phase3 on xRed
if flag_distFT
    disturbFT.Data(idxPh3,SpotCoord.xRed) = disturbFT_xRed;  
end

% start the simulation
myEvent.Value = 'suppressHardwareWarning';
myApp.public_StartSimulationButtonPushed([]);

% sanity check: measurement error
figure('name','Output Signal (run 1)')
plot(dataClass.Time_s(      idxPh3(1:end-1) ), ... 
     dataClass.RED_PROCx_m( idxPh3(2:end)   ), ...
     dataClass.Time_s(      idxPh3(1:end-1) ), ... 
     dataClass.RED_Px_m(    idxPh3(1:end-1) ));
legend('RED\_PROCx\_m', 'RED\_Px\_m', 'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_*x\_m');
title('output signal');

% sanity check: actuation error
figure('name','Control Signal (run 1)')
plot(dataClass.Time_s(        idxPh3(1:end-1) ), ...
     dataClass.RED_Fx_Real_N( idxPh3(2:end)   ), ... 
     dataClass.Time_s(        idxPh3(1:end-1) ), ...
     dataClass.RED_Fx_Sat_N(  idxPh3(2:end)   ), ...
     dataClass.Time_s(        idxPh3(1:end-1) ), ...
     dataClass.RED_Fx_N(      idxPh3(1:end-1) ));
legend('RED\_Fx\_Real\_N', 'RED\_Fx\_Sat\_N', 'RED\_Fx\_N', 'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_*x\_N');
title('control signal');

% sanity check: velocity estimator
figure('name','Velocity Estimator (run 1)')
plot(dataClass.Time_s(idxPh3), ...
     dataClass.RED_Vx_Est_mpers(idxPh3), ... 
     dataClass.Time_s(idxPh3), ...
     dataClass.RED_Vx_mpers(idxPh3));
legend('RED\_Vx\_Est\_mpers', 'RED\_Vx\_mpers', 'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_*x\_mpers');
title('output rate');

% sanity check: bias estimator
figure('name','Bias Estimator (run 1)')
plot(dataClass.Time_s(idxPh3), ...
     dataClass.RED_BIASx_Est_mpers2(idxPh3), ... 
     disturbFT.Time(idxPh3,SpotCoord.xRed), ...
     disturbFT.Data(idxPh3,SpotCoord.xRed) / mRED);
legend('RED\_BIASx\_Est\_mpers2', 'disturbFT', 'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_*x\_mpers2');
title('bias disturbance');

% extract the relevant data
tVec  = dataClass.Time_s(      idxPh3_ILC );
rVec  = dataClass.RED_REFx_m(  idxPh3_ILC );
uVec1 = dataClass.RED_Fx_N(    idxPh3_ILC ) / mRED;
yVec1 = dataClass.RED_PROCx_m( idxPh3_ILC );


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
uTilde1 = uVec1 - uVec0;
yTilde1 = yVec1 - yVec0;

% [ETHZ] calculate modelling error estimate and updated input 
d0    = G\yTilde1 - F*uTilde1 - G\H*uTilde1;
uNew0 = -F\d0;


%% KALMAN FILTER

% convert standard deviations into variances
walkFT_xRed_var    = ( walkFT_xRed_3sig    / 3 )^2;
noiseFT_xRed_var   = ( noiseFT_xRed_3sig   / 3 )^2;
noiseMeas_xRed_var = ( noiseMeas_xRed_3sig / 3 )^2;

% P0 - initial model error variance
% we construct this assuming a disturbance three times the actual
posVec = 3 * 0.5 * disturbFT_xRed / mRED * ( tVec - tVec(1) ).^2;
velVec = 3 *       disturbFT_xRed / mRED * ( tVec - tVec(1) );

dHat0 = zeros( xDim*nSteps, 1 );
dHat0(1:xDim:end) = posVec;
dHat0(2:xDim:end) = velVec;

P0_0 = diag( dHat0.^2 );

% dHat0 - initial model error
% we follow the advice given in the paper and set this to zero
dHat0 = 0*dHat0;

% Omega - random walk covariance
% Xi - process noise covariance
% Upsilon - measurement noise covariance
OmegaVec   = ones( nSteps, 1 ) * walkFT_xRed_var    / mRED^2;
XiVec      = ones( nSteps, 1 ) * noiseFT_xRed_var   / mRED^2;
UpsilonVec = ones( nSteps, 1 ) * noiseMeas_xRed_var;

Omega   = F * diag(OmegaVec)   * F';
Xi      = F * diag(XiVec)      * F';
Upsilon =     diag(UpsilonVec);

% M - stochastic variable covariance
M = G*Xi*G' + Upsilon;

% I - identity matrix
I = eye( xDim*nSteps );

% kalman filter equations
P1_0  = P0_0 + Omega;
Theta = G*P1_0*G' + M;
K     = P1_0 * G' / Theta;
P1_1  = (I - K*G) * P1_0;
d1    = dHat0 + K * ( yTilde1 - G*dHat0 - ( G*F + H ) * uTilde1 );

% minimization problem (2-norm)
uNew1 = quadprog( F'*F, d1'*F );


%% KALMAN PERFORMANCE

% break out data 
diagP  = diag(P1_1);
sigPos = sqrt(diagP(1:2:end));
d0Pos  =         d0(1:2:end);
d1Pos  =         d1(1:2:end);
sigVel = sqrt(diagP(2:2:end));
d0Vel  =         d0(2:2:end);
d1Vel  =         d1(2:2:end);

% first element
figure('Name', 'Kalman (I)');
plot(tVec, d0Pos, tVec, d1Pos, tVec, d1Pos+sigPos, 'm--', tVec, d1Pos-sigPos, 'm--');
legend('raw estimate','filter estimate','filter uncertainty','Location','BestOutside');
xlabel('Time\_s');
ylabel('1st element');
title('kalman performance');

% second element
figure('Name', 'Kalman (II)');
plot(tVec, d0Vel, tVec, d1Vel, tVec, d1Vel+sigVel, 'm--', tVec, d1Vel-sigVel, 'm--');
legend('raw estimate','filter estimate','filter uncertainty','Location','BestOutside');
xlabel('Time\_s');
ylabel('2nd element');
title('kalman performance');

% new command
figure('Name', 'Kalman (III)');
plot(tVec, uNew0, tVec, uNew1);
legend('raw estimate','filter estimate','Location','BestOutside');
xlabel('Time\_s');
ylabel('disturbance compensation');
title('kalman performance');


%% SIMULATION 2: WITH COMPENSATION

% select a feed-forward signal
if flag_kalman
    uNew = uNew1;
else
    uNew = uNew0;
end

% resample the updated input as a feed-forward force in Phase3;
% interpolation and extrapolation using the 'previous' method
feedForward_xRed = interp1(tVec, mRED*uNew, feedForward.Time(idxPh3), 'previous', 'extrap');

% update the feed-forward signal
paramCtrl(SpotPhase.Phase3_4,SpotCoord.xRed).fun = SpotGnc.ctrlPdFwd_vel;
paramCtrl(SpotPhase.Phase3_4,SpotCoord.xRed).k4  = 1;
feedForward.Data(     idxPh3,SpotCoord.xRed)     = feedForward_xRed;

% apply a random walk to the (initially uniform) disturbance
if flag_walkFT
    disturbFT.Data(idxPh3,SpotCoord.xRed) = disturbFT.Data(idxPh3,SpotCoord.xRed) + ...
        (walkFT_xRed_3sig / 3) * randn( length(idxPh3), 1 );
end

% start the simulation
myEvent.Value = 'suppressHardwareWarning';
myApp.public_StartSimulationButtonPushed([]);

% extract the relevant data
uVec2 = dataClass.RED_Fx_N(    idxPh3_ILC ) / mRED;
yVec2 = dataClass.RED_PROCx_m( idxPh3_ILC );

uFbk2 = (dataClass.RED_Fx_Kp_N(idxPh3_ILC) + dataClass.RED_Fx_Kd_N(idxPh3_ILC) ) / mRED;
uFwd2 =  dataClass.RED_Fx_u0_N(idxPh3_ILC) / mRED;

%% SIMULATION 3: FURTHER IMPROVEMENTS?

% repeat the command update
uTilde2 = uVec2 - uVec0;
yTilde2 = yVec2 - yVec0;

if flag_kalman
    P2_1  = P1_1 + Omega;
    Theta = G*P2_1*G' + M;
    K     = P2_1 * G' / Theta;
    P2_2  = (I - K*G) * P2_1;
    d2    = d1 + K * ( yTilde2 - G*d1 - ( G*F + H ) * uTilde2 );
    uNew2 = quadprog( F'*F, d2'*F );
else
    d2 = G\yTilde2 - F*uTilde2 - G\H*uTilde2;
    uNew2 = -F\d2;
end

feedForward_xRed = interp1(tVec, mRED*uNew2, feedForward.Time(idxPh3), 'previous', 'extrap');
feedForward.Data(idxPh3,SpotCoord.xRed) = feedForward_xRed;

% apply a random walk to the disturbance
% (for now, it's still a constant)
if flag_walkFT
    disturbFT.Data(idxPh3,SpotCoord.xRed) = disturbFT.Data(idxPh3,SpotCoord.xRed) + ...
        (walkFT_xRed_3sig / 3) * randn( length(idxPh3), 1 );
end

% start the simulation
myEvent.Value = 'suppressHardwareWarning';
myApp.public_StartSimulationButtonPushed([]);

% extract the relevant data
uVec3 = dataClass.RED_Fx_N(    idxPh3_ILC ) / mRED;
yVec3 = dataClass.RED_PROCx_m( idxPh3_ILC );


%% PLOT

% output signal
figure('name','RED_Px_m (coord)')
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
figure('name','RED_Fx_N')
plot(tVec,mRED * [uVec0 uVec1 uVec2 uFbk2 uFwd2 uVec3]);
hold on;
plot(disturbFT.Time(idxPh3,SpotCoord.xRed),-1 * disturbFT.Data(idxPh3,SpotCoord.xRed),'k--')
legend('nominal','1: feedback', '2: feedback + feedfwd', ...
       '2: feedback', '2: feedfwd', '3: further improvement', ...
       [char(8211) '1 \times disturbance'], 'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_Fx\_N');
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

