package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.function.Supplier;
import java.util.stream.Stream;

@Config
public class Drivetrain extends RobotModule {

    public static double TRACKWIDTH_CM = 50;
    public static double WHEEL_DIAMETER_CM = 10.6;
    private static final int MOTOR_CPR_NOGEARBOX = 24;
    private static final double MOTOR_GEARING = 10;
    public static double MOTOR_CPR = MOTOR_CPR_NOGEARBOX * MOTOR_GEARING;

    public static DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static DcMotor.ZeroPowerBehavior MOTOR_ZEROPOWERBEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

    private DcMotorEx leftMainMotor;
    private DcMotorEx rightMainMotor;
    private DcMotorEx leftAuxMotor;
    private DcMotorEx rightAuxMotor;

    private Supplier<Stream<DcMotorEx>> allMotors = () -> Stream.of(
            leftMainMotor,
            leftAuxMotor,
            rightMainMotor,
            rightAuxMotor
    );

    private void setLeftMotorPower(double power) {
        leftMainMotor.setPower(power);
        leftAuxMotor.setPower(power);
    }

    private void setRightMotorPower(double power) {
        rightMainMotor.setPower(power);
        rightAuxMotor.setPower(power);
    }

    private int getLeftMotorPosition() {
        return leftMainMotor.getCurrentPosition();
    }

    private int getRightMotorPosition() {
        return rightMainMotor.getCurrentPosition();
    }

    private double getLeftMotorVelocity() {
        return leftMainMotor.getVelocity();
    }

    private double getRightMotorVelocity() {
        return rightMainMotor.getVelocity();
    }

    public void setRawPower(double rawForwardPower, double rawTurnPower) {
        this.rawForwardPower = rawForwardPower;
        this.rawTurnPower = rawTurnPower;
    }

    private double rawForwardPower = 0.0;
    private double rawTurnPower = 0.0;

    public Drivetrain(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        leftMainMotor = robot.hardware.leftMainMotor;
        leftAuxMotor = robot.hardware.leftAuxMotor;

        rightMainMotor = robot.hardware.rightMainMotor;
        rightAuxMotor = robot.hardware.rightAuxMotor;

        leftMainMotor.setDirection(LEFT_MOTOR_DIRECTION);
        leftAuxMotor.setDirection(LEFT_MOTOR_DIRECTION);
        rightMainMotor.setDirection(RIGHT_MOTOR_DIRECTION);
        rightAuxMotor.setDirection(RIGHT_MOTOR_DIRECTION);

        allMotors.get().forEach(dcMotor -> dcMotor.setZeroPowerBehavior(MOTOR_ZEROPOWERBEHAVIOR));
        allMotors.get().forEach(dcMotor -> dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        allMotors.get().forEach(dcMotor -> dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    @Override
    public void update() {
        setLeftMotorPower(rawForwardPower - rawTurnPower);
        setRightMotorPower(rawForwardPower + rawTurnPower);
    }
}
