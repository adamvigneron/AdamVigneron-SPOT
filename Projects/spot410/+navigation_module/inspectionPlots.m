%% PRE-SETUP

% clear SpotEstimator
% 
% for i = 2:length(dataClass_rt.Time_s.Time)
%     [~,~,~,debugOut] = SpotEstimator( dataClass_rt.SpotGnc_Phase.Data(i,:), ...
%                                       dataClass_rt.SpotGnc_Proc.Data(i,:), ...
%                                       dataClass_rt.SpotGnc_Cmd.Data(i-1,:), ...
%                                       paramEst);
% 
%     dataClass_rt.SpotGnc_EkfDebug.Data(i,:) = debugOut(1,:);
% end


%% SETUP

% change default figure style
set(groot,'DefaultFigureWindowStyle','docked');
set(groot,'DefaultLineLineWidth', 1.5);
set(groot,'DefaultAxesFontSize', 12);
set(groot,'defaultAxesXGrid','on');
set(groot,'defaultAxesYGrid','on');
set(groot,'defaultAxesFontName','Times')

colorMap = orderedcolors('gem');


%% DATA

[~,SpotCoordNames]  = enumeration(SpotCoord(1));
[~,SpotSensorNames] = enumeration(SpotSensor(1));

Phase   = timeseries2timetable(dataClass_rt.SpotGnc_Phase);

Proc    = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_Proc),   1,'NewVariableNames',SpotSensorNames);

CtrlFwd = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_CtrlFwd),1,'NewVariableNames',SpotCoordNames);
CtrlKd  = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_CtrlKd), 1,'NewVariableNames',SpotCoordNames);
CtrlKp  = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_CtrlKp), 1,'NewVariableNames',SpotCoordNames);
Cmd     = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_Cmd),    1,'NewVariableNames',SpotCoordNames);
Err     = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_Err),    1,'NewVariableNames',SpotCoordNames);
ErrVel  = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_ErrVel), 1,'NewVariableNames',SpotCoordNames);
Est     = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_Est),    1,'NewVariableNames',SpotCoordNames);
EstBias = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_EstBias),1,'NewVariableNames',SpotCoordNames);
EstVel  = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_EstVel), 1,'NewVariableNames',SpotCoordNames);
Ref     = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_Ref),    1,'NewVariableNames',SpotCoordNames);
RefVel  = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_RefVel), 1,'NewVariableNames',SpotCoordNames);


%% CONSTRUCT RELATIVE MEASUREMENTS

ProcRel = timetable(Proc.Time);
ProcRel.xInertial     = Proc.xBlackPhasespace     - Proc.xRedPhasespace;
ProcRel.yInertial     = Proc.yBlackPhasespace     - Proc.yRedPhasespace;
ProcRel.thetaInertial = Proc.thetaBlackPhasespace - Proc.thetaRedPhasespace;

ProcRel.range         = sqrt( ProcRel.xInertial.^2 + ProcRel.yInertial.^2 );

ProcRel.xRateInertial     = Proc.xBlackRatePhasespace     - Proc.xRedRatePhasespace;
ProcRel.yRateInertial     = Proc.yBlackRatePhasespace     - Proc.yRedRatePhasespace;
ProcRel.thetaRateInertial = Proc.thetaBlackRatePhasespace - Proc.thetaRedRatePhasespace;

ProcRel.xRateRotating = ProcRel.xRateInertial + ProcRel.yInertial .* Proc.thetaRedRatePhasespace;
ProcRel.yRateRotating = ProcRel.yRateInertial - ProcRel.xInertial .* Proc.thetaRedRatePhasespace;


%% ROTATE TO BODY FRAME

ProcRel.xBody = 0 * ProcRel.xInertial;
ProcRel.yBody = 0 * ProcRel.yInertial;

ProcRel.xRateBody = 0 * ProcRel.xRateInertial;
ProcRel.yRateBody = 0 * ProcRel.yRateInertial;

CmdRel = timetable(Cmd.Time);

CmdRel.xRedBody = 0 * Cmd.xRed;
CmdRel.yRedBody = 0 * Cmd.yRed;

for i = 1:length(Proc.Time)

    theta = Proc.thetaRedPhasespace(i);

    ProcRel.xBody(i) = ProcRel.xInertial(i) * cos(theta) + ProcRel.yInertial(i) * sin(theta);
    ProcRel.yBody(i) = ProcRel.yInertial(i) * cos(theta) - ProcRel.xInertial(i) * sin(theta);

    ProcRel.xRateBody(i) = ProcRel.xRateRotating(i) * cos(theta) + ProcRel.yRateRotating(i) * sin(theta);
    ProcRel.yRateBody(i) = ProcRel.yRateRotating(i) * cos(theta) - ProcRel.xRateRotating(i) * sin(theta);

    CmdRel.xRedBody(i) = Cmd.xRed(i) * cos(theta) + Cmd.yRed(i) * sin(theta);
    CmdRel.yRedBody(i) = Cmd.yRed(i) * cos(theta) - Cmd.xRed(i) * sin(theta);

end


%% BREAK OUT SENSOR DATA

% lidar
ProcRel.rangeLidar = sqrt( Proc.xLidar.^2 + Proc.yLidar.^2 );

% stereo camera
ProcRel.rangeStereo = sqrt( Proc.xStereo.^2 + Proc.yStereo.^2 );

% laser rangefinder
ProcRel.rangeLrf = Proc.rLaser / 100;  % centimetres to metres


%% BREAK OUT KALMAN FILTER ESTIMATES

Ekf = timetable(Proc.Time);

Ekf.xBr          = dataClass_rt.SpotGnc_EkfDebug.Data(:,1);
Ekf.yBr          = dataClass_rt.SpotGnc_EkfDebug.Data(:,2);
Ekf.thetaBr      = dataClass_rt.SpotGnc_EkfDebug.Data(:,3);
Ekf.thetaRed     = dataClass_rt.SpotGnc_EkfDebug.Data(:,4);
Ekf.xBrRate      = dataClass_rt.SpotGnc_EkfDebug.Data(:,5);
Ekf.yBrRate      = dataClass_rt.SpotGnc_EkfDebug.Data(:,6);
Ekf.thetaBrRate  = dataClass_rt.SpotGnc_EkfDebug.Data(:,7);
Ekf.thetaRedRate = dataClass_rt.SpotGnc_EkfDebug.Data(:,8);

Ekf.xBr3sig          = 3*sqrt(dataClass_rt.SpotGnc_EkfDebug.Data(:,9));
Ekf.yBr3sig          = 3*sqrt(dataClass_rt.SpotGnc_EkfDebug.Data(:,18));
Ekf.thetaBr3sig      = 3*sqrt(dataClass_rt.SpotGnc_EkfDebug.Data(:,27));
Ekf.thetaRed3sig     = 3*sqrt(dataClass_rt.SpotGnc_EkfDebug.Data(:,36));
Ekf.xBrRate3sig      = 3*sqrt(dataClass_rt.SpotGnc_EkfDebug.Data(:,45));
Ekf.yBrRate3sig      = 3*sqrt(dataClass_rt.SpotGnc_EkfDebug.Data(:,54));
Ekf.thetaBrRate3sig  = 3*sqrt(dataClass_rt.SpotGnc_EkfDebug.Data(:,63));
Ekf.thetaRedRate3sig = 3*sqrt(dataClass_rt.SpotGnc_EkfDebug.Data(:,72));

Ekf.range = sqrt( Ekf.xBr.^2 + Ekf.yBr.^2 );


%% SENSOR PLOTS

cuRed = '#e91c24';

figure;
plot(seconds(Proc.Time), ProcRel.xBody, 'k');
hold on;
plot(seconds(Proc.Time), Proc.xLidar, 'b');
plot(seconds(Proc.Time), 0*Proc.xLidar + 0.85, 'Color', '#808080', 'LineStyle', '--')
plot(seconds(Proc.Time), Ekf.xBr, 'Color', cuRed);
plot(seconds(Proc.Time), Ekf.xBr+Ekf.xBr3sig,'Color',cuRed,'LineStyle',':')
plot(seconds(Proc.Time), Ekf.xBr-Ekf.xBr3sig,'Color',cuRed,'LineStyle',':')
xlabel('time, s');
ylabel('relative x-axis, m');
legend('phasespace','lidar','','filter');
xlim([145 325]);
ylim([0.65 1.1]);

% mysize = [20 10]; set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', mysize, 'PaperPosition', [0 0 mysize]);
% print('xBody','-dpdf','-r200')

figure;
plot(seconds(Proc.Time), ProcRel.yBody, 'k');
hold on;
plot(seconds(Proc.Time), Proc.yLidar, 'b');
plot(seconds(Proc.Time), 0*Proc.yLidar, 'Color', '#808080', 'LineStyle', '--')
plot(seconds(Proc.Time), Ekf.yBr, 'Color', cuRed);
plot(seconds(Proc.Time), Ekf.yBr+Ekf.yBr3sig,'Color',cuRed,'LineStyle',':')
plot(seconds(Proc.Time), Ekf.yBr-Ekf.yBr3sig,'Color',cuRed,'LineStyle',':')
xlabel('time, s');
ylabel('relative y-axis, m')
legend('phasespace','lidar','','filter');
xlim([145 325]);
ylim([-0.2 0.25]);

% mysize = [20 10]; set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', mysize, 'PaperPosition', [0 0 mysize]);
% print('yBody','-dpdf','-r200')

idxLidar = ( diff( Proc.xLidar ) ~= 0 );
idxLidar = [ false; idxLidar(1:end-1) ];

figure;
plot(Proc.Time, Proc.thetaBlackPhasespace-pi/2, 'k');
hold on;
stairs(Proc.Time(idxLidar), rad2deg(Proc.thetaLidar(idxLidar) + Proc.thetaRedPhasespace(idxLidar) ), 'b', 'LineWidth', 2);
plot(seconds(Proc.Time), rad2deg(Ekf.thetaBr + Proc.thetaRedPhasespace), 'Color', cuRed);
plot(Proc.Time, rad2deg(Ekf.thetaBr + Proc.thetaRedPhasespace+Ekf.thetaBr3sig),'Color',cuRed,'LineStyle',':')
plot(Proc.Time, rad2deg(Ekf.thetaBr + Proc.thetaRedPhasespace-Ekf.thetaBr3sig),'Color',cuRed,'LineStyle',':')
xlabel('time, s');
ylabel('thetaBlack, degrees');
legend('phasespace','lidar','filter');
xlim([seconds(145) seconds(325)]);

% mysize = [20 10]; set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', mysize, 'PaperPosition', [0 0 mysize]);
% print('thetaBlack','-dpdf','-r200')

figure;
plot(Proc.Time, rad2deg(Proc.thetaRedPhasespace-Ref.thetaRed));
hold on;
plot(seconds(Proc.Time), rad2deg(Ekf.thetaRed - Ref.thetaRed));
plot(Proc.Time, rad2deg(Ekf.thetaRed - Ref.thetaRed+Ekf.thetaRed3sig),'Color',colorMap(2,:),'LineStyle',':')
plot(Proc.Time, rad2deg(Ekf.thetaRed - Ref.thetaRed-Ekf.thetaRed3sig),'Color',colorMap(2,:),'LineStyle',':')
xlabel('time, s');
ylabel('thetaRed w.r.t. reference, degrees');
legend('phasespace','filter');

% mysize = [20 10]; set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', mysize, 'PaperPosition', [0 0 mysize]);
% print('thetaBlack','-dpdf','-r200')

figure;
plot(ProcRel.Time, [ProcRel.xRateBody Ekf.xBrRate]);
hold on;
plot(Proc.Time,Ekf.xBrRate+Ekf.xBrRate3sig,'Color',colorMap(2,:),'LineStyle',':')
plot(Proc.Time,Ekf.xBrRate-Ekf.xBrRate3sig,'Color',colorMap(2,:),'LineStyle',':')
xlabel('time, s');
ylabel('SpotCoord.xRed, m/s');
legend('phasespace','ekf');
% ylim([-0.05 0.05]);

figure;
plot(ProcRel.Time, [ProcRel.yRateBody Ekf.yBrRate]);
hold on;
plot(Proc.Time,Ekf.yBrRate+Ekf.yBrRate3sig,'Color',colorMap(2,:),'LineStyle',':')
plot(Proc.Time,Ekf.yBrRate-Ekf.yBrRate3sig,'Color',colorMap(2,:),'LineStyle',':')
xlabel('time, s');
ylabel('SpotCoord.yRed, m/s');
legend('phasespace','ekf');
% ylim([-0.05 0.05]);

figure;
plot(Proc.Time, rad2deg([ProcRel.thetaRateInertial Ekf.thetaBrRate Proc.thetaRedImu]))
hold on;
plot(Proc.Time,rad2deg(Ekf.thetaBrRate+Ekf.thetaBrRate3sig),'Color',colorMap(2,:),'LineStyle',':')
plot(Proc.Time,rad2deg(Ekf.thetaBrRate-Ekf.thetaBrRate3sig),'Color',colorMap(2,:),'LineStyle',':')
xlabel('time, s');
ylabel('SpotCoord.thetaRed, degree/s')
legend('phasespace','ekf','imu');

figure;
plot(Proc.Time, rad2deg([Proc.thetaRedRatePhasespace Ekf.thetaRedRate Proc.thetaRedImu]))
hold on;
plot(Proc.Time,rad2deg(Ekf.thetaRedRate+Ekf.thetaRedRate3sig),'Color',colorMap(2,:),'LineStyle',':')
plot(Proc.Time,rad2deg(Ekf.thetaRedRate-Ekf.thetaRedRate3sig),'Color',colorMap(2,:),'LineStyle',':')
xlabel('time, s');
ylabel('inertial rotation, degree/s')
legend('phasespace','ekf','imu');


%% CONTROL PLOTS

figure;
plot(Est.Time, [Est.xRed Ref.xRed Err.xRed]);
xlabel('time, s');
ylabel('SpotCoord.xRed, m');
legend('est','ref','err');
% ylim([-0.2 1.2]);

figure;
plot(Est.Time, [Est.yRed Ref.yRed Err.yRed]);
xlabel('time, s');
ylabel('SpotCoord.yRed, m');
legend('est','ref','err');
% ylim([-0.7 0.7]);

figure;
plot(Est.Time, rad2deg([Est.thetaRed Ref.thetaRed Err.thetaRed]));
xlabel('time, s');
ylabel('SpotCoord.thetaRed, degrees');
legend('est','ref','err');

figure;
plot(EstVel.Time, [EstVel.xRed RefVel.xRed ErrVel.xRed]);
xlabel('time, s');
ylabel('SpotCoord.xRed, m/s');
legend('estVel','refVel','errVel');

figure;
plot(EstVel.Time, [EstVel.yRed RefVel.yRed ErrVel.yRed]);
xlabel('time, s');
ylabel('SpotCoord.yRed, m/s');
legend('estVel','refVel','errVel');

figure;
plot(EstVel.Time, rad2deg([EstVel.thetaRed RefVel.thetaRed ErrVel.thetaRed]));
xlabel('time, s');
ylabel('SpotCoord.thetaRed, degree/s');
legend('estVel','refVel','errVel');

figure;
plot(Cmd.Time, [CtrlKp.xRed CtrlKd.xRed Cmd.xRed CtrlFwd.xRed]);
xlabel('time, s');
ylabel('SpotCoord.xRed, m/s²');
legend('Kp','Kd','total','fwd');

figure;
plot(Cmd.Time, [CtrlKp.yRed CtrlKd.yRed Cmd.yRed]);
xlabel('time, s');
ylabel('SpotCoord.yRed, m/s²');
legend('Kp','Kd','total');

figure;
plot(Cmd.Time, rad2deg([CtrlKp.thetaRed CtrlKd.thetaRed Cmd.thetaRed]));
xlabel('time, s');
ylabel('SpotCoord.thetaRed, degree/s²');
legend('Kp','Kd','total');

figure;
plot(dataClass_rt.RED_Fx_N);
hold on;
plot(dataClass_rt.RED_Fx_Sat_N);
xlabel('time, s');
ylabel('SpotCoord.xRed, N');
legend('RED\_Fx\_N','RED\_Fx\_Sat\_N');

figure;
plot(dataClass_rt.RED_Fy_N);
hold on;
plot(dataClass_rt.RED_Fy_Sat_N);
xlabel('time, s');
ylabel('SpotCoord.yRed, N');
legend('RED\_Fy\_N','RED\_Fy\_Sat\_N');

figure;
plot(dataClass_rt.RED_Tz_Nm);
hold on;
plot(dataClass_rt.RED_Tz_Sat_Nm);
xlabel('time, s');
ylabel('SpotCoord.thetaRed, Nm');
legend('RED\_Tz\_Nm','RED\_Tz\_Sat\_Nm');


%% TRAJECTORY

figure;
plot(Proc.xRedPhasespace, Proc.yRedPhasespace,'r');
hold on;
plot(Proc.xBlackPhasespace, Proc.yBlackPhasespace, 'k');
plot(Proc.xBlackPhasespace(end) + 0.85*cosd(0:359), Proc.yBlackPhasespace(end) + 0.85*sind(0:359),'Color','#808080','LineStyle','--');
xlabel('SPOT x-axis');
ylabel('SPOT y-axis');
legend('RED','BLACK');
axis square;
xlim([0 3]);
ylim([0 3]);

figure;
plot(Phase.Time,Phase.SpotGnc_Phase);

plotTimes = timerange(seconds(145),seconds(325));

xLidarRms = rms(ProcRel(plotTimes,:).xBody - Proc(plotTimes,:).xLidar);
yLidarRms = rms(ProcRel(plotTimes,:).yBody - Proc(plotTimes,:).yLidar);

[xLidarRms yLidarRms]
