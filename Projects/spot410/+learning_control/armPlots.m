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


%% THETA PLOTS

figure;
plot(Proc.Time,rad2deg([Ref.thetaRed Proc.thetaRedPhasespace Est.thetaRed Err.thetaRed]));
legend('Ref','Proc','Est','Err')
title('*.thetaRed');

figure;
plot(Err.Time, rad2deg(Err.thetaRed));
hold on
plot(Err.Time-seconds(120),  rad2deg(Err.thetaRed));
plot(Err.Time-seconds(240), rad2deg(Err.thetaRed));
plot(Err.Time-seconds(360), rad2deg(Err.thetaRed));
plot([seconds(30) seconds(150)],[0 0], 'k');
xlim([seconds(30) seconds(150)]);
legend('first','second','third','fourth')
title('Err.thetaRed');

rms1 = (rms(Err(timerange(seconds( 60),seconds( 80)),:).thetaRed) + rms(Err(timerange(seconds(110),seconds(130)),:).thetaRed) ) / 2;
rms2 = (rms(Err(timerange(seconds(180),seconds(200)),:).thetaRed) + rms(Err(timerange(seconds(230),seconds(250)),:).thetaRed) ) / 2;
rms3 = (rms(Err(timerange(seconds(300),seconds(320)),:).thetaRed) + rms(Err(timerange(seconds(350),seconds(370)),:).thetaRed) ) / 2;
rms4 = (rms(Err(timerange(seconds(420),seconds(440)),:).thetaRed) + rms(Err(timerange(seconds(470),seconds(490)),:).thetaRed) ) / 2;
thetaRms = rad2deg([rms1 rms2 rms3 rms4])

figure;
plot(ErrVel.Time, [RefVel.thetaRed EstVel.thetaRed ErrVel.thetaRed])
hold on;
plot(dataClass_rt.RED_RzD_radpers);
legend('RefVel','EstVel','ErrVel','RED\_RzD');
title('*.thetaRed');

figure;
plot(CtrlKp.Time, [CtrlKp.thetaRed CtrlKd.thetaRed CtrlFwd.thetaRed]);
legend('CtrlKp','CtrlKd','CtrlFwd')
title('*.thetaRed');

figure;
plot(CtrlKp.Time, CtrlKp.thetaRed+CtrlKd.thetaRed);
hold on
plot(CtrlKp.Time-seconds(120),  CtrlKp.thetaRed+CtrlKd.thetaRed+CtrlFwd.thetaRed);
plot(CtrlKp.Time-seconds(240), CtrlKp.thetaRed+CtrlKd.thetaRed+CtrlFwd.thetaRed);
plot(CtrlKp.Time-seconds(360), CtrlKp.thetaRed+CtrlKd.thetaRed+CtrlFwd.thetaRed);
xlim([seconds(30) seconds(150)]);
legend('first','second','third','fourth')
title('Ctrl*.thetaRed');

figure; 
plot(dataClass_rt.RED_Tz_Nm);
hold on
plot(dataClass_rt.RED_Tz_Sat_Nm);
legend('Tz','Sat');
title('RED_Tz_Nm');


%% X PLOTS

figure;
plot(Proc.Time,[Ref.xRed Proc.xRedPhasespace Est.xRed Err.xRed]);
legend('Ref','Proc','Est','Err')
title('*.xRed');

figure;
plot(Err.Time, Err.xRed);
hold on
plot(Err.Time-seconds(120),  Err.xRed);
plot(Err.Time-seconds(240), Err.xRed);
plot(Err.Time-seconds(360), Err.xRed);
plot([seconds(30) seconds(150)],[0 0], 'k');
xlim([seconds(30) seconds(150)]);
legend('first','second','third','fourth')
title('Err.xRed');

figure;
plot(ErrVel.Time, [RefVel.xRed EstVel.xRed ErrVel.xRed])
hold on;
plot(dataClass_rt.RED_Vx_mpers);
legend('RefVel','EstVel','ErrVel','RED\_Vx');
title('*.xRed');

figure;
plot(CtrlKp.Time, [CtrlKp.xRed CtrlKd.xRed CtrlFwd.xRed]);
legend('CtrlKp','CtrlKd','CtrlFwd')
title('*.xRed');

figure;
plot(CtrlKp.Time, CtrlKp.xRed+CtrlKd.xRed);
hold on
plot(CtrlKp.Time-seconds(120),  CtrlKp.xRed+CtrlKd.xRed+CtrlFwd.xRed);
plot(CtrlKp.Time-seconds(240), CtrlKp.xRed+CtrlKd.xRed+CtrlFwd.xRed);
plot(CtrlKp.Time-seconds(360), CtrlKp.xRed+CtrlKd.xRed+CtrlFwd.xRed);
xlim([seconds(30) seconds(150)]);
legend('first','second','third','fourth')
title('Ctrl*.xRed');

figure; 
plot(dataClass_rt.RED_Fx_N);
hold on
plot(dataClass_rt.RED_Fx_Sat_N);
legend('Fx','Sat');
title('RED_Fx_N');


%% Y PLOTS

figure;
plot(Proc.Time,[Ref.yRed Proc.yRedPhasespace Est.yRed Err.yRed]);
legend('Ref','Proc','Est','Err')
title('*.yRed');

figure;
plot(Err.Time, Err.yRed);
hold on
plot(Err.Time-seconds(120),  Err.yRed);
plot(Err.Time-seconds(240), Err.yRed);
plot(Err.Time-seconds(360), Err.yRed);
plot([seconds(30) seconds(150)],[0 0], 'k');
xlim([seconds(30) seconds(150)]);
legend('first','second','third','fourth')
title('Err.yRed');

figure;
plot(ErrVel.Time, [RefVel.yRed EstVel.yRed ErrVel.yRed])
hold on;
plot(dataClass_rt.RED_Vy_mpers);
legend('RefVel','EstVel','ErrVel','RED\_Vy');
title('*.yRed');

figure;
plot(CtrlKp.Time, [CtrlKp.yRed CtrlKd.yRed CtrlFwd.yRed]);
legend('CtrlKp','CtrlKd','CtrlFwd')
title('*.yRed');

figure;
plot(CtrlKp.Time, CtrlKp.yRed+CtrlKd.yRed);
hold on
plot(CtrlKp.Time-seconds(120),  CtrlKp.yRed+CtrlKd.yRed+CtrlFwd.yRed);
plot(CtrlKp.Time-seconds(240), CtrlKp.yRed+CtrlKd.yRed+CtrlFwd.yRed);
plot(CtrlKp.Time-seconds(360), CtrlKp.yRed+CtrlKd.yRed+CtrlFwd.yRed);
xlim([seconds(30) seconds(150)]);
legend('first','second','third','fourth')
title('Ctrl*.yRed');

figure; 
plot(dataClass_rt.RED_Fy_N);
hold on
plot(dataClass_rt.RED_Fy_Sat_N);
legend('Fy','Sat');
title('RED_Fy_N');


%% ARM PLOTS

figure;
plot(Proc.Time, [Proc.shoulderArmEncoder Proc.elbowArmEncoder Proc.wristArmEncoder])
legend('shoulder','elbow','wrist');
title('Proc.*ArmEncoder')