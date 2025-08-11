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


%% CONSTRUCT RELATIVE MEASUREMENTS

ProcRel = timetable(Proc.Time);
ProcRel.xInertial     = Proc.xBlackPhasespace     - Proc.xRedPhasespace;
ProcRel.yInertial     = Proc.yBlackPhasespace     - Proc.yRedPhasespace;
ProcRel.thetaInertial = Proc.thetaBlackPhasespace - Proc.thetaRedPhasespace;

ProcRel.xRateInertial     = Proc.xBlackRatePhasespace     - Proc.xRedRatePhasespace;
ProcRel.yRateInertial     = Proc.yBlackRatePhasespace     - Proc.yRedRatePhasespace;
ProcRel.thetaRateInertial = Proc.thetaBlackRatePhasespace - Proc.thetaRedRatePhasespace;

ProcRel.range         = sqrt( ProcRel.xInertial.^2 + ProcRel.yInertial.^2 );

ProcRel.xBody = 0 * ProcRel.xInertial;
ProcRel.yBody = 0 * ProcRel.yInertial;

ProcRel.xRateBody = 0 * ProcRel.xRateInertial;
ProcRel.yRateBody = 0 * ProcRel.yRateInertial;

for i = 1:length(Proc.Time)
    theta = Proc.thetaRedPhasespace(i);

    ProcRel.xBody(i) = ProcRel.xInertial(i) * cos(theta) + ProcRel.yInertial(i) * sin(theta);
    ProcRel.yBody(i) = ProcRel.yInertial(i) * cos(theta) - ProcRel.xInertial(i) * sin(theta);

    ProcRel.xRateBody(i) = ProcRel.xRateInertial(i) * cos(theta) + ProcRel.yRateInertial(i) * sin(theta);
    ProcRel.yRateBody(i) = ProcRel.yRateInertial(i) * cos(theta) - ProcRel.xRateInertial(i) * sin(theta);

end


%% PLOT

figure;
plot(ProcRel.Time, ProcRel.xBody);
hold on;
plot(Proc.Time, Proc.xStereo);
plot(EkfDebug.Time, EkfDebug.Data(:,1));
xlabel('time, s');
ylabel('relative x, m');

figure;
plot(ProcRel.Time, ProcRel.yBody);
hold on;
plot(Proc.Time, Proc.yStereo);
plot(EkfDebug.Time, EkfDebug.Data(:,2));
xlabel('time, s');
ylabel('relative y, m');
ylim([-0.7 0.7])

figure;
plot(ProcRel.Time, wrapTo2Pi(ProcRel.thetaInertial));
hold on;
plot(Proc.Time, wrapTo2Pi(Proc.thetaStereo));
plot(EkfDebug.Time, wrapTo2Pi(EkfDebug.Data(:,3)));
xlabel('time, s');
ylabel('relative theta, rad');

figure;
plot(ProcRel.Time, ProcRel.xRateBody);
hold on;
plot(EkfDebug.Time, movmean(EkfDebug.Data(:,4),3));
xlabel('time, s');
ylabel('relative xDot, m/s');

figure;
plot(ProcRel.Time, ProcRel.yRateBody);
hold on;
plot(EkfDebug.Time, EkfDebug.Data(:,5));
plot([0 250],0.85*0.03490659*[1 1]);
xlabel('time, s');
ylabel('relative yDot, m/s');

figure;
plot(ProcRel.Time, ProcRel.thetaRateInertial);
hold on;
plot(EkfDebug.Time, EkfDebug.Data(:,6));
xlabel('time, s');
ylabel('relative thetaDot, rad/s');
