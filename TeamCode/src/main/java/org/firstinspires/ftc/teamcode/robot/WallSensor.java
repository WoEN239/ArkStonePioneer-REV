package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TimedQuery;
import org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor.AsyncRev2MSensor;

@Config
public class WallSensor extends RobotModule {

    public static volatile double MIN_DISTANCE_TO_WALL_M = 0.15;
    public static volatile DistanceMode DISTANCE_MODE = DistanceMode.AVERAGE;
    public static volatile AsyncRev2MSensor.AccuracyMode ACCURACY_MODE = AsyncRev2MSensor.AccuracyMode.MODE_HIGH_ACCURACY;
    private boolean nearWall = false;
    private double distanceM = 2.5;
    private DistanceSensor leftDistanceSensor;
    public static double SENSOR_REFRESH_RATE_HZ = 6.66;//50
    private final TimedQuery<Double> distanceSensor1Query = new TimedQuery<>(() -> leftDistanceSensor.getDistance(DistanceUnit.METER), SENSOR_REFRESH_RATE_HZ);
    private DistanceSensor rightDistanceSensor;
    private final TimedQuery<Double> distanceSensor2Query = new TimedQuery<>(() -> rightDistanceSensor.getDistance(DistanceUnit.METER), SENSOR_REFRESH_RATE_HZ);

    public WallSensor(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        DistanceSensor distanceSensor1 = robot.hardware.leftDistanceSensor;
        DistanceSensor distanceSensor2 = robot.hardware.rightDistanceSensor;
        leftDistanceSensor = distanceSensor1;
        rightDistanceSensor = distanceSensor2;
        //if (distanceSensor1 instanceof Rev2mDistanceSensor)
        //    leftDistanceSensor = new AsyncRev2MSensor((Rev2mDistanceSensor) distanceSensor1);
        //if (distanceSensor2 instanceof Rev2mDistanceSensor)
        //    rightDistanceSensor = new AsyncRev2MSensor((Rev2mDistanceSensor) distanceSensor2);
        //leftDistanceSensor.setSensorAccuracyMode(ACCURACY_MODE);
        //rightDistanceSensor.setSensorAccuracyMode(ACCURACY_MODE);
        //leftDistanceSensor.enable();
        //rightDistanceSensor.enable();
    }

    @Override
    public void update() {
        distanceM = DISTANCE_MODE.equals(DistanceMode.MIN) ?
                Math.min(distanceSensor1Query.getValue(), distanceSensor2Query.getValue())
                : (distanceSensor1Query.getValue() + distanceSensor2Query.getValue()) / 2;
        nearWall = distanceM < MIN_DISTANCE_TO_WALL_M;
    }

    public double getDistanceM() {
        return distanceM;
    }

    public boolean isNearWall() {
        return nearWall;
    }

    public enum DistanceMode {
        MIN, AVERAGE
    }
}
