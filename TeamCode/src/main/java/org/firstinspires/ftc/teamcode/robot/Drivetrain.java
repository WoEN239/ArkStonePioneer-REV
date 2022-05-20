package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.CoupledDcMotorEx;
import org.firstinspires.ftc.teamcode.util.PIDVASMotorController;
import org.firstinspires.ftc.teamcode.util.TimedSender;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;
import java.util.stream.Stream;

@Config
public class Drivetrain extends RobotModule {

    private static final int MOTOR_CPR_NOGEARBOX = 24;
    private static final double MOTOR_GEARING = 10.0;
    private static final double TRACK_WIDTH_CM = 33.70002;
    private static final double WHEEL_DIAMETER_CM = 10.6;
    private static final double MOTOR_CPR = MOTOR_CPR_NOGEARBOX * MOTOR_GEARING;
    private static final double ENCODER_TICKS_TO_CM_RATIO = (WHEEL_DIAMETER_CM * PI) / (MOTOR_CPR);
    private static final double CM_TO_ROTATION_RADIANS_RATIO = 1.0 / TRACK_WIDTH_CM;
    private static final double MAX_MOTOR_RPM = 6000.0;
    private static final double MAX_MOTOR_TPS = MOTOR_CPR * MAX_MOTOR_RPM / 60.0;
    public static volatile DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static volatile DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static volatile DcMotor.ZeroPowerBehavior MOTOR_ZEROPOWERBEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;
    public static volatile double MOTOR_REFRESH_RATE_HZ = 0.5;
    public static volatile DrivetrainControlMode DRIVETRAIN_CONTROL_MODE = DrivetrainControlMode.FEEDFORWARD;
    public static volatile double VELOCITY_KP = 27.0;
    public static volatile double VELOCITY_KI = 0.5;
    public static volatile double VELOCITY_KD = .0;
    public static volatile double VELOCITY_KV = 14.46;
    public static volatile double VELOCITY_KS = 2400.0;
    public static volatile double VELOCITY_MAXI = 16536;
    private DcMotorEx leftMotor;
    private final TimedSender<Double> leftMotorPowerSender = new TimedSender<>(power -> leftMotor.setPower(power), MOTOR_REFRESH_RATE_HZ);
    private final DoubleConsumer leftMotorVoltageCompensator = power -> leftMotorPowerSender.trySend(power * robot.batteryVoltageSensor.getKVoltage());
    private PIDVASMotorController leftMotorController;
    private DcMotorEx rightMotor;
    private final TimedSender<Double> rightMotorPowerSender = new TimedSender<>(power -> rightMotor.setPower(power), MOTOR_REFRESH_RATE_HZ);
    private final DoubleConsumer rightMotorVoltageCompensator = power -> rightMotorPowerSender.trySend(power * robot.batteryVoltageSensor.getKVoltage());
    private PIDVASMotorController rightMotorController;
    private double forwardVelocity = 0.0;
    private double turnVelocity = 0.0;

    public Drivetrain(WoENRobot robot) {
        super(robot);
    }

    private double cmToEncoderTicks(double cm) {
        return cm / ENCODER_TICKS_TO_CM_RATIO;
    }

    private double encoderTicksToCm(double ticks) {
        return ticks * ENCODER_TICKS_TO_CM_RATIO;
    }

    private double encoderTicksToRotationRadians(double ticks) {
        return encoderTicksToCm(ticks) * CM_TO_ROTATION_RADIANS_RATIO;
    }

    private double motorEncodersToDistance(double leftTicks, double rightTicks) {
        return encoderTicksToCm((leftTicks + rightTicks) * 0.5);
    }

    private double motorEncodersToRotation(double leftTicks, double rightTicks) {
        return encoderTicksToRotationRadians((-leftTicks + rightTicks) * 0.5);
    }

    public void setRawPower(double rawForwardPower, double rawTurnPower) {
        this.forwardVelocity = rawForwardPower;
        this.turnVelocity = rawTurnPower;
    }

    @Override
    public void initialize() {
        leftMotor = new CoupledDcMotorEx(robot.hardware.leftMainMotor, robot.hardware.leftAuxMotor);
        rightMotor = new CoupledDcMotorEx(robot.hardware.rightMainMotor, robot.hardware.rightAuxMotor);

        leftMotor.setDirection(LEFT_MOTOR_DIRECTION);
        rightMotor.setDirection(RIGHT_MOTOR_DIRECTION);

        Supplier<Stream<DcMotorEx>> allMotors = () -> Stream.of(leftMotor, rightMotor);

        allMotors.get().forEach(dcMotor -> dcMotor.setZeroPowerBehavior(MOTOR_ZEROPOWERBEHAVIOR));
        allMotors.get().forEach(dcMotor -> dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        allMotors.get().forEach(dcMotor -> dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        leftMotorController = new PIDVASMotorController(leftMotorVoltageCompensator, leftMotor::getVelocity,
                () -> VELOCITY_KP, () -> VELOCITY_KI, () -> VELOCITY_KD, () -> VELOCITY_KV, () -> 0, () -> VELOCITY_KS, () -> VELOCITY_MAXI, 0, false);
        rightMotorController = new PIDVASMotorController(leftMotorVoltageCompensator, rightMotor::getVelocity,
                () -> VELOCITY_KP, () -> VELOCITY_KI, () -> VELOCITY_KD, () -> VELOCITY_KV, () -> 0, () -> VELOCITY_KS, () -> VELOCITY_MAXI, 0, false);
    }

    @Override
    public void update() {
        if (DRIVETRAIN_CONTROL_MODE == DrivetrainControlMode.FEEDFORWARD) {
            leftMotorVoltageCompensator.accept(forwardVelocity - turnVelocity);
            rightMotorVoltageCompensator.accept(forwardVelocity + turnVelocity);
        } else {
            leftMotorController.update(MAX_MOTOR_TPS * (forwardVelocity - turnVelocity));
            rightMotorController.update(MAX_MOTOR_TPS * (forwardVelocity + turnVelocity));
        }
    }

    public enum DrivetrainControlMode {
        FEEDFORWARD, PIDVAS
    }
}
