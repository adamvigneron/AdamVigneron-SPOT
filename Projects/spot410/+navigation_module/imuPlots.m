t = dataClass_rt.Time_s.Data;

xAccelPhasespace = dataClass_rt.RED_Ax_mpers2.Data;
yAccelPhasespace = dataClass_rt.RED_Ay_mpers2.Data;
zGyroPhasespace  = dataClass_rt.RED_RzD_radpers.Data;

xAccelImu = dataClass_rt.RED_IMU_Ax_mpers2.Data;
yAccelImu = dataClass_rt.RED_IMU_Ay_mpers2.Data;
zGyroImu  = dataClass_rt.RED_IMU_Gz_radpers.Data;

figure;
plot(t, xAccelImu - xAccelImu(2), 'r', t, yAccelImu - yAccelImu(2), 'b');
hold on;
plot(t, xAccelPhasespace, 'r--', t, yAccelPhasespace, 'b--');
legend('xImu', 'yImu', 'xPhasespace', 'yPhasespace');
xlabel('time, s');
ylabel('acceleration, m/s^2');
grid on;

figure;
plot(t, zGyroImu, 'b');
hold on;
plot(t, zGyroPhasespace, 'b--');
legend('zImu', 'zPhasespace');
xlabel('time, s');
ylabel('rotational rate, rad/s');
grid on;

