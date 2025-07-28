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

Phase   = timeseries2timetable(dataClass_rt.SpotGnc_Phase);

Proc    = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_Proc),   1,'NewVariableNames',SpotSensorNames);

CtrlFwd = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_CtrlFwd),1,'NewVariableNames',SpotCoordNames);
CtrlKd  = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_CtrlKd), 1,'NewVariableNames',SpotCoordNames);
CtrlKp  = splitvars(timeseries2timetable(dataClass_rt.SpotGnc_CtrlKp), 1,'NewVariableNames',SpotCoordNames);
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

ProcRel.xBody = 0 * ProcRel.xInertial;
ProcRel.yBody = 0 * ProcRel.yInertial;

for i = 1:length(Proc.Time)
    theta = Proc.thetaRedPhasespace(i);
    ProcRel.xBody(i) = ProcRel.xInertial(i) * cos(theta) + ProcRel.yInertial(i) * sin(theta);
    ProcRel.yBody(i) = ProcRel.yInertial(i) * cos(theta) - ProcRel.xInertial(i) * sin(theta);
end


%% BREAK OUT SENSOR DATA

% lidar
ProcRel.xLidar     = dataClass_rt.SpotGnc_Lidar1.Data(:,1);
ProcRel.yLidar     = dataClass_rt.SpotGnc_Lidar1.Data(:,2);
ProcRel.thetaLidar = dataClass_rt.SpotGnc_Lidar1.Data(:,3);
ProcRel.rmseLidar  = dataClass_rt.SpotGnc_Lidar1.Data(:,4);

ProcRel.rangeLidar = sqrt( ProcRel.xLidar.^2 + ProcRel.yLidar.^2 );

% stereo camera
ProcRel.xStereo     = dataClass_rt.SpotGnc_Cnn.Data(:,1);
ProcRel.yStereo     = dataClass_rt.SpotGnc_Cnn.Data(:,2);
ProcRel.thetaStereo = dataClass_rt.SpotGnc_Cnn.Data(:,3);
ProcRel.confStereo  = dataClass_rt.SpotGnc_Cnn.Data(:,4);

ProcRel.rangeStereo = sqrt( ProcRel.xStereo.^2 + ProcRel.yStereo.^2 );

% laser rangefinder
ProcRel.rangeLrf = dataClass_rt.SpotGnc_Lrf.Data / 100;  % centimetres to metres


%% PLOT

figure;
plot(ProcRel.Time,[ProcRel.xBody ProcRel.xLidar ProcRel.xStereo])
xlabel('time, s');
ylabel('x-position, m')
title('body frame');
legend('phasespace', 'lidar', 'stereo');

figure;
plot(ProcRel.Time,[ProcRel.yBody ProcRel.yLidar ProcRel.yStereo])
xlabel('time, s');
ylabel('y-position, m')
title('body frame');
legend('phasespace', 'lidar', 'stereo');
ylim([-0.7 0.7])

figure;
plot(ProcRel.Time,rad2deg([ProcRel.thetaInertial ProcRel.thetaLidar ProcRel.thetaStereo]))
xlabel('time, s');
ylabel('theta-rotation, rad')
title('body frame');
legend('phasespace', 'lidar', 'stereo');

figure;
plot(ProcRel.Time,[ProcRel.range ProcRel.rangeLidar ProcRel.rangeStereo], ProcRel.Time, ProcRel.rangeLrf, '.')
xlabel('time, s');
ylabel('range, m')
title('body frame');
legend('phasespace', 'lidar', 'stereo', 'rangefinder');
ylim([0 2])

figure;
plot(dataClass_rt.RED_IMU_Ax_mpers2);
hold on;
plot(dataClass_rt.RED_IMU_Ay_mpers2);
plot(dataClass_rt.RED_IMU_Gz_radpers);
legend('Ax','Ay','Gz');

xLidarRms = rms(ProcRel(timerange(seconds(55),seconds(225)),:).xBody - ProcRel(timerange(seconds(55),seconds(225)),:).xLidar);
yLidarRms = rms(ProcRel(timerange(seconds(55),seconds(225)),:).yBody - ProcRel(timerange(seconds(55),seconds(225)),:).yLidar);

xStereoRms = rms(ProcRel(timerange(seconds(55),seconds(225)),:).xBody - ProcRel(timerange(seconds(55),seconds(225)),:).xStereo);
yStereoRms = rms(ProcRel(timerange(seconds(55),seconds(225)),:).yBody - ProcRel(timerange(seconds(55),seconds(225)),:).yStereo);

[xLidarRms yLidarRms; xStereoRms yStereoRms]
