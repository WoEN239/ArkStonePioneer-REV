package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Drivetrain extends RobotModule {

    public static double TRACKWIDTH_CM = 50;
    public static double WHEEL_DIAMETER_CM = 10.6;
    private static final int MOTOR_CPR_NOGEARBOX = 24;
    private static final double MOTOR_GEARING = 10;
    public static double MOTOR_CPR = MOTOR_CPR_NOGEARBOX * MOTOR_GEARING;

    public Drivetrain(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void update() {

    }
}
