%% SETUP

% change default figure style
set(groot,'DefaultFigureWindowStyle','docked');
set(groot,'DefaultLineLineWidth', 2);
set(groot,'DefaultAxesFontSize', 12);
set(groot,'defaultAxesXGrid','on');
set(groot,'defaultAxesYGrid','on');


%% DATA

[~,SpotCoordNames]  = enumeration(SpotCoord(1));
[~,SpotSensorNames] = enumeration(SpotSensor(1));

Phase   = timeseries2timetable(dataClass.SpotGnc_Phase);

Proc    = splitvars(timeseries2timetable(dataClass.SpotGnc_Proc),   1,'NewVariableNames',SpotSensorNames);

CtrlFwd = splitvars(timeseries2timetable(dataClass.SpotGnc_CtrlFwd),1,'NewVariableNames',SpotCoordNames);
CtrlKd  = splitvars(timeseries2timetable(dataClass.SpotGnc_CtrlKd), 1,'NewVariableNames',SpotCoordNames);
CtrlKp  = splitvars(timeseries2timetable(dataClass.SpotGnc_CtrlKp), 1,'NewVariableNames',SpotCoordNames);
Err     = splitvars(timeseries2timetable(dataClass.SpotGnc_Err),    1,'NewVariableNames',SpotCoordNames);
ErrVel  = splitvars(timeseries2timetable(dataClass.SpotGnc_ErrVel), 1,'NewVariableNames',SpotCoordNames);
Est     = splitvars(timeseries2timetable(dataClass.SpotGnc_Est),    1,'NewVariableNames',SpotCoordNames);
EstBias = splitvars(timeseries2timetable(dataClass.SpotGnc_EstBias),1,'NewVariableNames',SpotCoordNames);
EstVel  = splitvars(timeseries2timetable(dataClass.SpotGnc_EstVel), 1,'NewVariableNames',SpotCoordNames);
Ref     = splitvars(timeseries2timetable(dataClass.SpotGnc_Ref),    1,'NewVariableNames',SpotCoordNames);
RefVel  = splitvars(timeseries2timetable(dataClass.SpotGnc_RefVel), 1,'NewVariableNames',SpotCoordNames);

EkfDebug = renamevars(timeseries2timetable(dataClass.SpotGnc_EkfDebug),'SpotGnc_EkfDebug','Data');


%% PLOT

figure;
plot(Proc.Time, Proc.xBlackPhasespace - Proc.xRedPhasespace);
hold on;
plot(EkfDebug.Time, EkfDebug.Data(:,1));
xlabel('time, s');
ylabel('relative x, m');

figure;
plot(Proc.Time, Proc.yBlackPhasespace - Proc.yRedPhasespace);
hold on;
plot(EkfDebug.Time, EkfDebug.Data(:,2));
xlabel('time, s');
ylabel('relative y, m');

figure;
plot(Proc.Time, wrapTo2Pi(Proc.thetaBlackPhasespace - Proc.thetaRedPhasespace));
hold on;
plot(EkfDebug.Time, wrapTo2Pi(EkfDebug.Data(:,3)));
xlabel('time, s');
ylabel('relative theta, rad');


figure;
plot(Proc.Time, Proc.xBlackRatePhasespace - Proc.xRedRatePhasespace);
hold on;
plot(EkfDebug.Time, movmean(EkfDebug.Data(:,4),3));
xlabel('time, s');
ylabel('relative xDot, m/s');

figure;
plot(Proc.Time, Proc.yBlackRatePhasespace - Proc.yRedRatePhasespace);
hold on;
plot(EkfDebug.Time, EkfDebug.Data(:,5));
xlabel('time, s');
ylabel('relative yDot, m/s');

figure;
plot(Proc.Time, Proc.thetaBlackRatePhasespace - Proc.thetaRedRatePhasespace);
hold on;
plot(EkfDebug.Time, EkfDebug.Data(:,6));
xlabel('time, s');
ylabel('relative thetaDot, rad/s');