%% INTRO

% "Iterative Learning Control of Spacecraft Proximity Operations Based on Confidence Level"
% Steve Ulrich and Kirk Hovell
% AIAA GNC 2017
% https://arc.aiaa.org/doi/10.2514/6.2017-1046
%
% This script simulates the numerical example of Section III.C


%% SETUP

% dock all figures by default
set(0,'DefaultFigureWindowStyle','docked');

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


%% PREDECLARE

betaVec = 0:0.1:0.9;
myTime  = 0:baseRate:tsim;

phase3 = (myTime > Phase2_End) & (myTime < Phase3_End);

nSim = length(betaVec);
nPts = length(myTime);

xRef = zeros(nSim,nPts);
xCmd = zeros(nSim,nPts);
xTra = zeros(nSim,nPts);
yRef = zeros(nSim,nPts);
yCmd = zeros(nSim,nPts);
yTra = zeros(nSim,nPts);
tRef = zeros(nSim,nPts);
tCmd = zeros(nSim,nPts);
tRot = zeros(nSim,nPts);

xCmd_Kp = zeros(nSim,nPts);
xCmd_Kd = zeros(nSim,nPts);
xCmd_u0 = zeros(nSim,nPts);
yCmd_Kp = zeros(nSim,nPts);
yCmd_Kd = zeros(nSim,nPts);
yCmd_u0 = zeros(nSim,nPts);
tCmd_Kp = zeros(nSim,nPts);
tCmd_Kd = zeros(nSim,nPts);
tCmd_u0 = zeros(nSim,nPts);


%% SIMULATE

for iSim = 1:nSim

    paramCtrl(SpotPhase.Phase3_4,SpotCoord.xRed).k4     = betaVec(iSim); %#ok<SAGROW>
    paramCtrl(SpotPhase.Phase3_4,SpotCoord.yRed).k4     = betaVec(iSim); %#ok<SAGROW>
    paramCtrl(SpotPhase.Phase3_4,SpotCoord.thetaRed).k4 = betaVec(iSim); %#ok<SAGROW>
    
    if exist('dataClass','var')
        feedForward.Data(:,SpotCoord.xRed)     = dataClass.RED_Fx_N;
        feedForward.Data(:,SpotCoord.yRed)     = dataClass.RED_Fy_N;
        feedForward.Data(:,SpotCoord.thetaRed) = dataClass.RED_Tz_Nm;
    end
    
    myEvent.Value = 'suppressHardwareWarning';
    myApp.public_StartSimulationButtonPushed(myEvent);

    xRef(iSim,:) = dataClass.RED_REFx_m;
    xCmd(iSim,:) = dataClass.RED_Fx_N;
    xTra(iSim,:) = dataClass.RED_Px_m;
    yRef(iSim,:) = dataClass.RED_REFy_m;
    yCmd(iSim,:) = dataClass.RED_Fy_N;
    yTra(iSim,:) = dataClass.RED_Py_m;
    tRef(iSim,:) = dataClass.RED_REFz_rad;
    tCmd(iSim,:) = dataClass.RED_Tz_Nm;
    tRot(iSim,:) = dataClass.RED_Rz_rad;

    xCmd_Kp(iSim,:) = dataClass.RED_Fx_Kp_N;
    xCmd_Kd(iSim,:) = dataClass.RED_Fx_Kd_N;
    xCmd_u0(iSim,:) = dataClass.RED_Fx_u0_N;

    yCmd_Kp(iSim,:) = dataClass.RED_Fy_Kp_N;
    yCmd_Kd(iSim,:) = dataClass.RED_Fy_Kd_N;
    yCmd_u0(iSim,:) = dataClass.RED_Fy_u0_N;

    tCmd_Kp(iSim,:) = dataClass.RED_Tz_Kp_Nm;
    tCmd_Kd(iSim,:) = dataClass.RED_Tz_Kd_Nm;
    tCmd_u0(iSim,:) = dataClass.RED_Tz_u0_Nm;

    posErr2D = mean( vecnorm( [ xRef(iSim,phase3) - xTra(iSim,phase3); ...
                                yRef(iSim,phase3) - yTra(iSim,phase3)] ) );
    disp( ['Iteration ' num2str(iSim,'%02i') ': posErr2D = ' num2str(posErr2D) ] );

end


%% PLOT

% trajectory
figure('Name','trajectory');
plot( xRef(1,phase3), yRef(1,phase3), 'k--', 'LineWidth', 2 );
hold on;
plot( xTra(:,phase3)', yTra(:,phase3)' )
title('trajectory');
xlabel('x, m');
ylabel('y, m');
legend('reference');
grid on;

% rotation
figure('Name','rotation');
plot( myTime(phase3), wrapToPi(tRef(1,phase3)), 'k--', 'LineWidth', 2 );
hold on;
plot( myTime(phase3), wrapToPi(tRot(:,phase3))')
title('rotation');
xlabel('theta, rad');
ylabel('time, s');
legend('reference');
grid on;

% x-axis control signal figure
figure('Name','x-control');
plot(myTime(phase3),xCmd(:,phase3));
title('control')
xlabel('time, s');
ylabel('f_{x,cmd}, N');
grid on;

% y-axis control signal figure
figure('Name','y-control');
plot(myTime(phase3),yCmd(:,phase3));
title('control')
xlabel('time, s');
ylabel('f_{y,cmd}, N');
grid on;

% theta-axis control signal figure
figure('Name','theta-control');
plot(myTime(phase3),tCmd(:,phase3));
title('control')
xlabel('time, s');
ylabel('t_{z,cmd}, Nm');
grid on;

% detail of x-control
figure('Name','x-detail');
subplot(1,3,1);
plot(myTime(phase3),xCmd_u0(:,phase3));
title('learning')
xlabel('time, s')
ylabel('f_x, N');
grid on;
subplot(1,3,2);
plot(myTime(phase3),xCmd_Kp(:,phase3));
title('proportional')
xlabel('time, s')
grid on;
subplot(1,3,3);
plot(myTime(phase3),xCmd_Kd(:,phase3));
title('derivative')
xlabel('time, s');
grid on;

% detail of y-control
figure('Name','y-detail');
subplot(1,3,1);
plot(myTime(phase3),yCmd_u0(:,phase3));
title('learning')
xlabel('time, s')
ylabel('f_y, N');
grid on;
subplot(1,3,2);
plot(myTime(phase3),yCmd_Kp(:,phase3));
title('proportional')
xlabel('time, s')
grid on;
subplot(1,3,3);
plot(myTime(phase3),yCmd_Kd(:,phase3));
title('derivative')
xlabel('time, s');
grid on;

% detail of theta-control
figure('Name','theta-detail');
subplot(1,3,1);
plot(myTime(phase3),tCmd_u0(:,phase3));
title('learning')
xlabel('time, s')
ylabel('t_z, Nm');
grid on;
subplot(1,3,2);
plot(myTime(phase3),tCmd_Kp(:,phase3));
title('proportional')
xlabel('time, s')
grid on;
subplot(1,3,3);
plot(myTime(phase3),tCmd_Kd(:,phase3));
title('derivative')
xlabel('time, s');
grid on;


%% SHUTDOWN

% reset figure style
set(0,'DefaultFigureWindowStyle','normal')

