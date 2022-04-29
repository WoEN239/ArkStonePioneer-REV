package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TimedQuery;

@Config
public class WallSensor extends RobotModule {

    public static double MIN_DISTANCE_TO_WALL_M = 0.15;

    private boolean nearWall = false;

    private double distanceM = 2.5;

    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;

    public static double SENSOR_REFRESH_RATE_HZ = 6;
    private final TimedQuery<Double> distanceSensor1Query = new TimedQuery<>(() -> distanceSensor1.getDistance(DistanceUnit.METER), SENSOR_REFRESH_RATE_HZ);
    private final TimedQuery<Double> distanceSensor2Query = new TimedQuery<>(() -> distanceSensor2.getDistance(DistanceUnit.METER), SENSOR_REFRESH_RATE_HZ);

    public WallSensor(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        distanceSensor1 = robot.hardware.leftDistanceSensor;
        distanceSensor2 = robot.hardware.rightDistanceSensor;
    }

    @Override
    public void update() {
        distanceM = (distanceSensor1Query.getValue() + distanceSensor2Query.getValue()) / 2;
        nearWall = distanceM < MIN_DISTANCE_TO_WALL_M;
    }

    public double getDistanceM() {
        return distanceM;
    }

    public boolean isNearWall() {
        return nearWall;
    }
}
