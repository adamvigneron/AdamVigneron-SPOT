%% INTRO

% AV: Loosely speaking, this script compares the seminal work on ILC [1] 
% with extensions in command feedback [2] and disturbance estimation [3] 
% as well as a disturbance-compensation alternative using a reduced-order
% estimator [4].
%
% [1] Arimoto, S., et al. (1984). Bettering Operation of Robots by
% Learning. Journal of Robotic Systems 1(2) 123-140.
%
% [2] Ulrich, S., et al. (2017). Iterative Learning Control of Spacecraft
% Proximity Operations Based on Confidence Level. AIAA SciTech, 9-13 Jan
% 2017, Grapevine TX, USA.
%
% [3] Schoellig, A., et al. (2009). Optimization-Based Iterative Learning
% Control for Trajectory Tracking. European Control Conference, 23-26 Aug
% 2009, Budapest, Hungary.
%
% [4] Bay, J. (1998). "State Feedback and Observers", in Fundamentals of 
% Linear State Space Systems. McGraw-Hill.


%% SIMULATION OR EXPERIMENT

% change default figure style
set(groot,'DefaultFigureWindowStyle','docked');
set(groot,'DefaultLineLineWidth', 2);
set(groot,'DefaultAxesFontSize', 18);
set(groot,'defaultAxesXGrid','on');
set(groot,'defaultAxesYGrid','on');

% run the SPOT intialization script
Run_Initializer;

% set the main switch, suppressing relevant warning
isExperiment = 0;  %#ok<*UNRCH>


%% SETUP

% get a handle for the open app via its containing figure
myFig = findall(groot,'Name','SPOT 4.0 GUI');
myApp = myFig.RunningAppInstance;

% select the first available simulink model
myApp.AvailableDiagramsDropDown.Value = myApp.AvailableDiagramsDropDown.Items{2};
myApp.public_AvailableDiagramsDropDownValueChanged([]);  % [] is an empty event

if isExperiment
    % activate only RED
    myApp.REDCheckBox.Value = 1; 
    myApp.BLUECheckBox.Value = 0;
    myApp.BLACKCheckBox.Value = 0;
else
    % activate all three platforms
    myApp.REDCheckBox.Value = 1;
    myApp.BLUECheckBox.Value = 1;
    myApp.BLACKCheckBox.Value = 1;
end

myApp.public_REDCheckBoxValueChanged([]);
myApp.public_BLUECheckBoxValueChanged([]);
myApp.public_BLACKCheckBoxValueChanged([]);

if isExperiment
    % merge Phase 1 into Phase 2 (not reflected in GUI)
    Phase2_Duration = Phase2_Duration + Phase1_Duration - 1;
    Phase1_Duration = 1;
    Phase1_End      = Phase0_End + 1;
end


%% CONFIGURATION

% choose ILC method
% 0 - none
% 1 - arimoto
% 2 - ulrich
% 3 - schoellig
% 4 - observer
ilcMethod = 4;

% noise switches
flag_distFT    = 1;
flag_walkFT    = 1;
flag_noiseFT   = 1;
flag_noiseMeas = 1;

% noise and disturbance values
disturbFT_xRed      = 0.1;     % newton, absolute
walkFT_xRed_3sig    = 0.02;    % newton, three-sigma
noiseFT_xRed_3sig   = 0.02;    % newton, three-sigma
noiseMeas_xRed_3sig = 0.0001;  % metre, three-sigma

% determine timeseries indices in Phase 3 for the model and the learning
idxPh2_End = round(Phase2_End / baseRate) + 2;
idxPh3_End = round(Phase3_End / baseRate) + 2;
idxPh3     = idxPh2_End:idxPh3_End;

% we run the learning algorithm two times slower than the model
sampleFactor = 2;
learnRate    = sampleFactor * baseRate;  % seconds


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

if ( flag_noiseMeas ) && ( ~isExperiment )
    noiseMeas.var(SpotCoord.xRed) = ( noiseMeas_xRed_3sig / 3 )^2;  
end


%% ETHZ MATRICES

% downsample the index vector
idxPh3_ETHZ = idxPh3(1:sampleFactor:end);
idxPh3_ETHZ = idxPh3_ETHZ(1:end-1);
nSteps     = length(idxPh3_ETHZ);

% F matrix
F_store        = zeros( xDim, uDim, nSteps);
F_store(:,:,2) = Bd;

for q = 3:nSteps
    F_store(:,:,q) = Ad * F_store(:,:,q-1);
end
    
F = zeros( nSteps*xDim, nSteps*uDim );

for l = 1:nSteps
    rowIdx = (1:xDim) + (l-1)*xDim;

    for m = 1:l
        colIdx = (1:uDim) + (m-1)*uDim;
        F(rowIdx,colIdx) = F_store(:,:,l-m+1);
    end
end

% G matrix
GCell = repmat({Cd},1,nSteps);
G     = blkdiag(GCell{:});


%% ROUND 1: WITHOUT COMPENSATION

% we apply a disturbance in Phase3 on xRed
if ( flag_distFT ) && ( ~isExperiment )
    disturbFT.Data(idxPh3,SpotCoord.xRed) = disturbFT_xRed;
end

if isExperiment
    % give the user a chance to react
    disp('Connect! PhaseSpace! Rebuild! Run! Save!');
    pause;
    disp('Run the experiment!');
    pause;
    disp('Save the data!');
    pause;

    % create a copy of the experimental data for the simulation plots
    dataClass = dataClass_rt;

    % as PhaseSpace is running at half the baseRate, we divide Vx by 2
    dataClass.RED_Vx_mpers(idxPh3) = dataClass.RED_Vx_mpers(idxPh3) / 2;

    % break out the CustomUserData
    dataClass.RED_REFx_m           = dataClass.CustomUserData48;
    dataClass.RED_PROCx_m          = dataClass.CustomUserData50;

    dataClass.RED_Vx_Est_mpers     = dataClass.CustomUserData52;

    dataClass.RED_Fx_N             = dataClass.CustomUserData54;
    dataClass.RED_Fx_Kp_N          = dataClass.CustomUserData55;
    dataClass.RED_Fx_Kd_N          = dataClass.CustomUserData56;
    dataClass.RED_Fx_u0_N          = dataClass.CustomUserData57;
    dataClass.RED_Fx_Real_N        = dataClass.CustomUserData58;
    dataClass.RED_BIASx_Est_mpers2 = dataClass.CustomUserData59;

    dataClass.Time_s               = dataClass.CustomUserData61;

else
    % start the simulation
    myEvent.Value = 'suppressHardwareWarning';
    myApp.public_StartSimulationButtonPushed([]);
end

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
tVec  = dataClass.Time_s(          idxPh3 );
rVec  = dataClass.RED_REFx_m(      idxPh3 );
uVec1 = dataClass.RED_Fx_N(        idxPh3 ) / mRED;
yVec1 = dataClass.RED_PROCx_m(     idxPh3 );
vVec1 = dataClass.RED_Vx_Est_mpers(idxPh3 );


%% COMMAND UPDATE - NOMINAL COMMAND

% nominal plant and controller model
tf_G = ss(A,B,C,D);
tf_K = pid( Kp_xr/mRED, 0, Kd_xr/mRED, (Kd_xr/Kp_xr)/20 );
% per [5], the smallest recommended time constant for a derivative filter;
% the time constant of 0.13 s corresponds to a frequency of 7.7 rad/s,
% our reference has a frequency of 0.035 rad/s, well below this value!
%
% [5] Aastrom, K.J., et al. (2020). Feedback Systems: an introduction for
% scientists and engineers. Princeton University Press, version 3.1.5.

% closed-loop transfer functions
tf_UR = feedback(tf_K,tf_G);
tf_YR = feedback(tf_K*tf_G,1);
tf_VR = tf_YR * tf('s');

% normalize closed-loop reference
if isExperiment  % as experimental rVec gets time signal from PhaseSpace,
                 % we need to avoid the step-like behaviour 
    k1 = paramRefGen(SpotPhase.Phase3_4,SpotCoord.xRed).k1;
    k2 = paramRefGen(SpotPhase.Phase3_4,SpotCoord.xRed).k2;
    rVec0 = k1 * cos( k2 * (tVec - tVec(1)) ) - k1;
else
    rVec0 = rVec - init_states_RED(1);
end

% map closed-loop reference onto control and output
uVec0 = lsim(tf_UR,rVec0,tVec);
yVec0 = lsim(tf_YR,rVec0,tVec) + init_states_RED(1);
vVec0 = lsim(tf_VR,rVec0,tVec);


%% COMMAND UPDATE - ARIMOTO

% least-squares the velocity estimate to get an acceleration estimate
spanTime = 1;  % second
spanPts  = round(spanTime / baseRate) + 1;
vVec1_smooth = smooth(vVec1,spanPts,'lowess');
aVec1 = [0; diff(vVec1_smooth)] / baseRate;

% sanity check: velocity smoothing
figure('name','Velocity Smoothing (run 1)')
plot(dataClass.Time_s(idxPh3), ...
     dataClass.RED_Vx_mpers(idxPh3) - vVec1, ... 
     dataClass.Time_s(idxPh3), ...
     dataClass.RED_Vx_mpers(idxPh3) - vVec1_smooth);
legend('estimator error', 'smoothed estimator error', ...
       'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_*x\_mpers');
title('output rate smoothing');

% sanity check: acceleration estimate
figure('name','Acceleration Estimate (run 1)')
plot(dataClass.Time_s(idxPh3), ...
     dataClass.RED_Fx_Real_N(idxPh3)/mRED, ... 
     tVec, aVec1);
legend('real acceleration', 'estimated acceleration', ...
       'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_*x\_mpers2');
title('output second derivative');


%% COMMAND UPDATE - SCHOELLIG

% zeroth-order ETHZ
tVec_ETHZ  = dataClass.Time_s(      idxPh3_ETHZ );
rVec_ETHZ  = dataClass.RED_REFx_m(  idxPh3_ETHZ );
uVec1_ETHZ = dataClass.RED_Fx_N(    idxPh3_ETHZ ) / mRED;
yVec1_ETHZ = dataClass.RED_PROCx_m( idxPh3_ETHZ );

% normalize closed-loop reference
if isExperiment  % as experimental rVec gets time signal from PhaseSpace,
                 % we need to avoid the step-like behaviour 
    k1 = paramRefGen(SpotPhase.Phase3_4,SpotCoord.xRed).k1;
    k2 = paramRefGen(SpotPhase.Phase3_4,SpotCoord.xRed).k2;
    rVec0_ETHZ = k1 * cos( k2 * (tVec_ETHZ - tVec_ETHZ(1)) ) - k1;
else
    rVec0_ETHZ = rVec_ETHZ - init_states_RED(1);
end

uVec0_ETHZ = lsim(tf_UR,rVec0_ETHZ,tVec_ETHZ);
yVec0_ETHZ = lsim(tf_YR,rVec0_ETHZ,tVec_ETHZ) + init_states_RED(1);

uETHZ_ETHZ = (uVec1_ETHZ - uVec0_ETHZ) - F \ ( G \ (yVec1_ETHZ - yVec0_ETHZ) );


%% COMMAND UPDATE - COMPARISON

% calculate all control signals for comparison
uNew_arimoto   = uVec1 + uVec0 - aVec1;
uNew_ulrich    = uVec1;
uNew_schoellig = interp1(tVec_ETHZ, uETHZ_ETHZ, tVec, 'previous', 'extrap');
uNew_observer  = -1 * dataClass.RED_BIASx_Est_mpers2(idxPh3);

% ilc signals
figure('name','feedForward_xRed')
plot(tVec, uVec0, '--', ...
     tVec, [ uNew_arimoto uNew_ulrich uNew_schoellig uNew_observer ] );
legend('model', 'arimoto', 'ulrich', 'schoellig', 'observer', ...
       'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_Fx\_u0\_mpers2');
title('feed forward signals');


%% COMMAND UPDATE - SELECTION

% choose compensation method
switch ilcMethod
    case 0
        uNew = 0 * tVec;
    case 1
        uNew = uNew_arimoto;
    case 2
        uNew = uNew_ulrich;
    case 3
        uNew = uNew_schoellig;
    case 4
        uNew = uNew_observer;
    otherwise
        error('ilcMethod undefined');
end

% resample the updated input as a feed-forward force in Phase3;
% interpolation and extrapolation using the 'previous' method
feedForward_xRed = interp1(tVec, mRED*uNew, feedForward.Time(idxPh3), 'previous', 'extrap');

% update the feed-forward signal
paramCtrl(SpotPhase.Phase3_4,SpotCoord.xRed).fun = SpotGnc.ctrlPdFwd_vel;
paramCtrl(SpotPhase.Phase3_4,SpotCoord.xRed).k4  = 1;
feedForward.Data(     idxPh3,SpotCoord.xRed)     = feedForward_xRed;


%% ROUND 2: WITH COMPENSATION

% apply a random walk to the (initially uniform) disturbance
if ( flag_walkFT ) && ( ~isExperiment ) 
    disturbFT.Data(idxPh3,SpotCoord.xRed) = disturbFT.Data(idxPh3,SpotCoord.xRed) + ...
        (walkFT_xRed_3sig / 3) * randn( length(idxPh3), 1 );
end

if isExperiment
    % give the user a chance to react
    disp('Rebuild the code!');
    pause;
    disp('Run the experiment!');
    pause;
    disp('Save the data!');
    pause;

    % create a copy of the experimental data for the simulation plots
    dataClass = dataClass_rt;
    
    dataClass.RED_REFx_m       = dataClass.CustomUserData48;
    dataClass.RED_PROCx_m      = dataClass.CustomUserData50;

    dataClass.RED_Vx_Est_mpers = dataClass.CustomUserData52;

    dataClass.RED_Fx_N         = dataClass.CustomUserData54;
    dataClass.RED_Fx_Kp_N      = dataClass.CustomUserData55;
    dataClass.RED_Fx_Kd_N      = dataClass.CustomUserData56;
    dataClass.RED_Fx_u0_N      = dataClass.CustomUserData57;
    dataClass.RED_Fx_Real_N    = dataClass.CustomUserData58;
    dataClass.RED_BIASx_Est    = dataClass.CustomUserData59;

else
    % start the simulation
    myEvent.Value = 'suppressHardwareWarning';
    myApp.public_StartSimulationButtonPushed([]);
end

% extract the relevant data
uVec2 = dataClass.RED_Fx_N(         idxPh3 ) / mRED;
yVec2 = dataClass.RED_PROCx_m(      idxPh3 );
vVec2 = dataClass.RED_Vx_Est_mpers( idxPh3 );

uFbk2 = (dataClass.RED_Fx_Kp_N(idxPh3) + dataClass.RED_Fx_Kd_N(idxPh3) ) / mRED;
uFwd2 =  dataClass.RED_Fx_u0_N(idxPh3) / mRED;


%% PLOT

% output signal
figure('name','RED_Px_m (coord)')
plot(tVec,[yVec0 yVec1 yVec2]);
legend('nominal','1: feedback', '2: feedback + feedfwd', ...
       'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_Px\_m');
title('output signal');

% output error
figure('name','RED_Px_m (error)')
plot(tVec,[yVec0-rVec yVec1-rVec yVec2-rVec]);
legend('nominal','1: feedback', '2: feedback + feedfwd', ...
       'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_Px\_m');
title('output error');

% control signal
figure('name','RED_Fx_N')
plot(tVec,mRED * [uVec0 uVec1 uVec2 uFbk2 uFwd2]);
hold on;
plot(disturbFT.Time(idxPh3,SpotCoord.xRed),-1 * disturbFT.Data(idxPh3,SpotCoord.xRed),'k--')
legend('nominal','1: feedback', '2: feedback + feedfwd', ...
       '2: feedback', '2: feedfwd', ...
       [char(8211) '1 \times disturbance'], 'Location', 'BestOutside');
xlabel('Time\_s');
ylabel('RED\_Fx\_N');
title('control signal');
ylim([-0.2 0.1]);

