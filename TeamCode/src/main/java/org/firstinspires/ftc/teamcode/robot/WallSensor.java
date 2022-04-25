package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class WallSensor extends RobotModule {

    public static double MIN_DISTANCE_TO_WALL_M = 0.15;

    private boolean nearWall = false;

    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;

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
        nearWall = (distanceSensor1.getDistance(DistanceUnit.METER) + distanceSensor2.getDistance(DistanceUnit.METER)) / 2
                < MIN_DISTANCE_TO_WALL_M;
    }

    public boolean isNearWall() {
        return nearWall;
    }
}
