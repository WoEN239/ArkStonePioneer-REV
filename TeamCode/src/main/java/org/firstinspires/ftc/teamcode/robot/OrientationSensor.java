package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;

import org.firstinspires.ftc.teamcode.util.TimedSensorQuery;

public class OrientationSensor extends RobotModule {

    public OrientationSensor(WoENRobot robot) {
        super(robot);
    }

    private BNO055IMU gyro;

    private final TimedSensorQuery<Float> timedGyroQuery = new TimedSensorQuery<>(() -> gyro
            .getAngularOrientation().firstAngle, 100);

    private float orientation = 0.0f;

    @Override
    public void initialize() {
        gyro = robot.hardware.imu;
        gyro.initialize(new BNO055IMU.Parameters());
        new LynxI2cDeviceSynchV2(robot.opMode.hardwareMap.appContext, robot.hardware.controlHub, 0)
                .setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        orientation = timedGyroQuery.getValue();
    }

    @Override
    public void update() {
        orientation = timedGyroQuery.getValue();
    }

    public float getOrientation() {
        return orientation;
    }
}