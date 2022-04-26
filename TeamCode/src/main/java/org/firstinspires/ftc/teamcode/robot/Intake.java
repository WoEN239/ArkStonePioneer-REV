package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends RobotModule {

    public static DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static double BRUSH_POWER = 0.75;

    private DcMotorEx leftBrushMotor;
    private DcMotorEx rightBrushMotor;

    public boolean isIntakeEnabled() {
        return enableIntake;
    }

    public void setEnableIntake(boolean enableIntake) {
        this.enableIntake = enableIntake;
    }

    private boolean enableIntake = false;

    public Intake(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        leftBrushMotor = robot.hardware.leftBrushMotor;
        rightBrushMotor = robot.hardware.rightBrushMotor;

        leftBrushMotor.setDirection(LEFT_MOTOR_DIRECTION);
        leftBrushMotor.setDirection(RIGHT_MOTOR_DIRECTION);

        leftBrushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBrushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBrushMotor.setPower(0);
        rightBrushMotor.setPower(0);
    }

    @Override
    public void update() {
        leftBrushMotor.setPower(enableIntake ? BRUSH_POWER : 0);
        rightBrushMotor.setPower(enableIntake ? BRUSH_POWER : 0);
    }
}
