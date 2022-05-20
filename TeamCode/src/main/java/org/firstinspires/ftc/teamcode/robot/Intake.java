package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.TimedSender;

import java.util.function.DoubleConsumer;

@Config
public class Intake extends RobotModule {

    public static volatile DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static volatile DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static volatile double BRUSH_POWER = 0.75;

    private DcMotorEx leftBrushMotor;
    private DcMotorEx rightBrushMotor;

    public static volatile double MOTOR_REFRESH_RATE_HZ = 0.1;

    private final TimedSender<Double> leftBrushMotorPowerSender = new TimedSender<>(power -> leftBrushMotor.setPower(power), MOTOR_REFRESH_RATE_HZ);
    private final TimedSender<Double> rightBrushMotorPowerSender = new TimedSender<>(power -> rightBrushMotor.setPower(power), MOTOR_REFRESH_RATE_HZ);

    private final DoubleConsumer leftBrushMotorVoltageCompensator = power -> leftBrushMotorPowerSender.trySend(power * robot.batteryVoltageSensor.getKVoltage());
    private final DoubleConsumer rightBrushMotorVoltageCompensator = power -> rightBrushMotorPowerSender.trySend(power * robot.batteryVoltageSensor.getKVoltage());


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
        rightBrushMotor.setDirection(RIGHT_MOTOR_DIRECTION);

        leftBrushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBrushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftBrushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBrushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBrushMotor.setPower(0);
        rightBrushMotor.setPower(0);
    }

    @Override
    public void update() {
        leftBrushMotorVoltageCompensator.accept(enableIntake ? BRUSH_POWER : 0);
        rightBrushMotorVoltageCompensator.accept(enableIntake ? BRUSH_POWER : 0);
    }
}
