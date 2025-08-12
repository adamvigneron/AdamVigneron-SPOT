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


% %% CONSTRUCT BODY FORCES
% 
% for i = 1:length(Ctrl.xRedInertial.Time)
%     theta = dataClass_rt.RED_Rz_rad(i);
% 
%     Ctrl.xRedBody(i) = Ctrl.xRedInertial(i) * cos(theta) + Ctrl.yRedInertial(i) * sin(theta);
%     Ctrl.yRedBody(i) = Ctrl.yRedInertial(i) * cos(theta) - Ctrl.xRedInertial(i) * sin(theta);
% end


%% PLOT

figure;
plot(ProcRel.Time, ProcRel.xBody,'k');
hold on;
plot(Proc.Time, Proc.xLidar,'Color','#808080');
plot(EkfDebug.Time, EkfDebug.Data(:,1),'Color','#e91c24');
plot(EkfDebug.Time, EkfDebug.Data(:,1) + 3*sqrt(EkfDebug.Data(:,8)),'Color','#e91c24','LineStyle','--');
plot(EkfDebug.Time, EkfDebug.Data(:,1) - 3*sqrt(EkfDebug.Data(:,8)),'Color','#e91c24','LineStyle','--');
xlabel('time, s');
ylabel('relative x, m');
legend('truth','meas','filter');

figure;
plot(ProcRel.Time, ProcRel.yBody,'k');
hold on;
plot(Proc.Time, Proc.yLidar,'Color','#808080');
plot(EkfDebug.Time, EkfDebug.Data(:,2),'Color','#e91c24');
plot(EkfDebug.Time, EkfDebug.Data(:,2) + 3*sqrt(EkfDebug.Data(:,16)),'Color','#e91c24','LineStyle','--');
plot(EkfDebug.Time, EkfDebug.Data(:,2) - 3*sqrt(EkfDebug.Data(:,16)),'Color','#e91c24','LineStyle','--');
xlabel('time, s');
ylabel('relative y, m');
legend('truth','meas','filter');
ylim([-0.7 0.7])

figure;
plot(ProcRel.Time, ProcRel.thetaInertial - ProcRel.thetaInertial(1),'k');
hold on;
plot(Proc.Time, Proc.thetaLidar,'Color','#808080');
plot(EkfDebug.Time, EkfDebug.Data(:,3),'Color','#e91c24');
plot(EkfDebug.Time, EkfDebug.Data(:,3) + 3*sqrt(EkfDebug.Data(:,24)),'Color','#e91c24','LineStyle','--');
plot(EkfDebug.Time, EkfDebug.Data(:,3) - 3*sqrt(EkfDebug.Data(:,24)),'Color','#e91c24','LineStyle','--');
xlabel('time, s');
ylabel('relative theta, m');
legend('truth','meas','filter');

figure;
plot(ProcRel.Time, ProcRel.xRateBody,'k');
hold on;
plot(EkfDebug.Time, EkfDebug.Data(:,4),'Color','#e91c24');
plot(EkfDebug.Time, EkfDebug.Data(:,4) + 3*sqrt(EkfDebug.Data(:,32)),'Color','#e91c24','LineStyle','--');
plot(EkfDebug.Time, EkfDebug.Data(:,4) - 3*sqrt(EkfDebug.Data(:,32)),'Color','#e91c24','LineStyle','--');
xlabel('time, s');
ylabel('relative xDot, m/s');
legend('truth','filter');

figure;
plot(ProcRel.Time, ProcRel.yRateBody,'k');
hold on;
plot(EkfDebug.Time, EkfDebug.Data(:,5),'Color','#e91c24');
plot(EkfDebug.Time, EkfDebug.Data(:,5) + 3*sqrt(EkfDebug.Data(:,40)),'Color','#e91c24','LineStyle','--');
plot(EkfDebug.Time, EkfDebug.Data(:,5) - 3*sqrt(EkfDebug.Data(:,40)),'Color','#e91c24','LineStyle','--');
plot([0 250],0.85*0.03490659*[1 1],'Color','#808080');
xlabel('time, s');
ylabel('relative yDot, m/s');
legend('truth','filter');

figure;
plot(ProcRel.Time, ProcRel.thetaRateInertial,'k');
hold on;
plot(EkfDebug.Time, EkfDebug.Data(:,6),'Color','#e91c24');
plot(EkfDebug.Time, EkfDebug.Data(:,6) + 3*sqrt(EkfDebug.Data(:,48)),'Color','#e91c24','LineStyle','--');
plot(EkfDebug.Time, EkfDebug.Data(:,6) - 3*sqrt(EkfDebug.Data(:,48)),'Color','#e91c24','LineStyle','--');
xlabel('time, s');
ylabel('relative thetaDot, rad/s');
legend('truth','filter');

figure;
plot(Proc.Time, Proc.thetaRedRatePhasespace,'k');
hold on;
plot(Proc.Time, Proc.thetaRedImu,'Color','#808080');
plot(EkfDebug.Time, EkfDebug.Data(:,7),'Color','#e91c24');
plot(EkfDebug.Time, EkfDebug.Data(:,7) + 3*sqrt(EkfDebug.Data(:,56)),'Color','#e91c24','LineStyle','--');
plot(EkfDebug.Time, EkfDebug.Data(:,7) - 3*sqrt(EkfDebug.Data(:,56)),'Color','#e91c24','LineStyle','--');
xlabel('time, s');
ylabel('omega, rad/s');
legend('truth','meas','filter');

% figure;
% plot(Ctrl.Time, Ctrl.xRedBody/11.2970);
% hold on;
% plot(Ctrl.Time, Ctrl.yRedBody/11.2970);

