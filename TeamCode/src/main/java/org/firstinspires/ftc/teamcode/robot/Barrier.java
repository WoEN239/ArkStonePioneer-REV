package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ServoEx;

@Config
public class Barrier extends RobotModule {

    private ServoEx barrierServo;

    public static double BARRIER_OPEN = 0.25;
    public static double BARRIER_CLOSE = 0.5;

    public static boolean USE_ANGLE = true;

    public static float minAngle = (float) toRadians(-15f);
    public static float maxAngle = (float) toRadians(105f);

    public Barrier(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        barrierServo = robot.hardware.barrierServo;
        barrierServo.setPwmEnable();
    }

    @Override
    public void update() {
        boolean open = robot.fieldSensor.isOnTeamSquare() && (!USE_ANGLE ||
                (minAngle < robot.orientationSensor.getOrientation() && robot.orientationSensor.getOrientation() < maxAngle));
        barrierServo.setPosition(open ? BARRIER_OPEN : BARRIER_CLOSE);
    }
}
