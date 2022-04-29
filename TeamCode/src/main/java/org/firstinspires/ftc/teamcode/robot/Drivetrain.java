package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.CoupledDcMotorEx;
import org.firstinspires.ftc.teamcode.util.TimedSender;

import java.util.function.Supplier;
import java.util.stream.Stream;

@Config
public class Drivetrain extends RobotModule {

    private static final int MOTOR_CPR_NOGEARBOX = 24;
    private static final double MOTOR_GEARING = 10;
    public static double TRACKWIDTH_CM = 50;
    public static double WHEEL_DIAMETER_CM = 10.6;
    public static double MOTOR_CPR = MOTOR_CPR_NOGEARBOX * MOTOR_GEARING;

    public static DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static DcMotor.ZeroPowerBehavior MOTOR_ZEROPOWERBEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    public static double MOTOR_REFRESH_RATE_HZ = 0.5;

    private final TimedSender<Double> leftMotorPowerSender = new TimedSender<>(power -> leftMotor.setPower(power), MOTOR_REFRESH_RATE_HZ);
    private final TimedSender<Double> rightMotorPowerSender = new TimedSender<>(power -> rightMotor.setPower(power), MOTOR_REFRESH_RATE_HZ);

    private final Supplier<Stream<DcMotorEx>> allMotors = () -> Stream.of(
            leftMotor,
            rightMotor
    );
    private double rawForwardPower = 0.0;
    private double rawTurnPower = 0.0;

    public Drivetrain(WoENRobot robot) {
        super(robot);
    }

    public void setRawPower(double rawForwardPower, double rawTurnPower) {
        this.rawForwardPower = rawForwardPower;
        this.rawTurnPower = rawTurnPower;
    }

    public double getForwardEncoderPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) * 0.5;
    }

    public double getRotationEncoderPosition() {
        return (leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition()) * 0.5;
    }

    @Override
    public void initialize() {
        leftMotor = new CoupledDcMotorEx(robot.hardware.leftMainMotor, robot.hardware.leftAuxMotor);
        rightMotor = new CoupledDcMotorEx(robot.hardware.rightMainMotor, robot.hardware.rightAuxMotor);

        leftMotor.setDirection(LEFT_MOTOR_DIRECTION);
        rightMotor.setDirection(RIGHT_MOTOR_DIRECTION);

        allMotors.get().forEach(dcMotor -> dcMotor.setZeroPowerBehavior(MOTOR_ZEROPOWERBEHAVIOR));
        allMotors.get().forEach(dcMotor -> dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        allMotors.get().forEach(dcMotor -> dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    @Override
    public void update() {
        leftMotorPowerSender.trySend(rawForwardPower - rawTurnPower);
        rightMotorPowerSender.trySend(rawForwardPower + rawTurnPower);
    }
}
