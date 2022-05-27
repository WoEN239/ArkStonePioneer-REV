package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;

import org.firstinspires.ftc.teamcode.robot.superclasses.RobotModule;
import org.firstinspires.ftc.teamcode.util.I2CUtils;
import org.firstinspires.ftc.teamcode.util.TimedQuery;

@Config
public class OrientationSensor extends RobotModule {

    public static volatile double SENSOR_REFRESH_RATE_HZ = 5;
    private static final LynxI2cDeviceSynch.BusSpeed I2C_BUS_SPEED = LynxI2cDeviceSynch.BusSpeed.FAST_400K;
    private BNO055IMU gyro;

    public boolean isNewDataAvailable() {
        return newDataAvailable;
    }

    private boolean newDataAvailable = false;
    private final TimedQuery<Float> timedGyroQuery = new TimedQuery<>(() -> {
        newDataAvailable = true;
        return gyro.getAngularOrientation().firstAngle;
    }, SENSOR_REFRESH_RATE_HZ);

    private float orientation = 0.0f;

    public OrientationSensor(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        gyro = robot.hardware.imu;
        gyro.initialize(new BNO055IMU.Parameters());
        if (gyro instanceof BNO055IMUImpl)
            I2CUtils.setBusSpeed((BNO055IMUImpl) gyro, I2C_BUS_SPEED); //TODO test it
        //new LynxI2cDeviceSynchV2(robot.opMode.hardwareMap.appContext, robot.hardware.controlHub, I2C_BUS).setBusSpeed(I2C_BUS_SPEED);
        orientation = timedGyroQuery.getValue();
    }

    @Override
    public void update() {
        newDataAvailable = false;
        orientation = timedGyroQuery.getValue();
    }

    public float getOrientation() {
        return orientation;
    }
}
