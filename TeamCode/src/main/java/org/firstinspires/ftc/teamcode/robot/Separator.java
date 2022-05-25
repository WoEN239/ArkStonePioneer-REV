package org.firstinspires.ftc.teamcode.robot;

import static android.graphics.Color.rgb;
import static org.firstinspires.ftc.teamcode.util.MathUtils.doubleEquals;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.color.ColorReference;
import org.firstinspires.ftc.teamcode.color.ColorReferenceMap;
import org.firstinspires.ftc.teamcode.color.FieldColor;
import org.firstinspires.ftc.teamcode.util.LowHighPassLimiter;
import org.firstinspires.ftc.teamcode.util.PIDVASMotorController;
import org.firstinspires.ftc.teamcode.util.TimedSender;

import java.util.EnumMap;
import java.util.function.DoubleConsumer;

@Config
public class Separator extends RobotModule {

    public static volatile DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static volatile DcMotor.ZeroPowerBehavior MOTOR_ZEROPOWERBEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    public static volatile double MOTOR_ROTATION_ENCODER_TICKS = 120 * 2; // Surprisingly LEGO motors have 720 CPR encoders
    public static volatile double MOTOR_STALL_DETECTION_S = 0.8;

    private DcMotorEx separatorMotor;

    private static final double MOTOR_CPR = 720;

    private static final double MOTOR_RPM_TO_TPS_RATIO = MOTOR_CPR /60.0;

    public static volatile double MOTOR_REFRESH_RATE_HZ = 0.5;

    private final TimedSender<Double> separatorMotorPowerSender = new TimedSender<>(power -> separatorMotor.setPower(power), MOTOR_REFRESH_RATE_HZ);

    private final DoubleConsumer separatorMotorVoltageCompensator = power -> separatorMotorPowerSender.trySend(power * robot.batteryVoltageSensor.getKVoltage());

    public static volatile double MIN_MOTOR_POWER = 0.1;
    public static volatile double MAX_MOTOR_POWER = 1.0;

    private final LowHighPassLimiter separatorMotorPowerLimiter = new LowHighPassLimiter(separatorMotorVoltageCompensator::accept, MIN_MOTOR_POWER, MAX_MOTOR_POWER);

    public static volatile double VELOCITY_KP = 5.0;
    public static volatile double VELOCITY_KI = 10.0;
    public static volatile double VELOCITY_KD = .0;
    public static volatile double VELOCITY_KV = 6.9;
    public static volatile double VELOCITY_KS = 4000.0;
    public static volatile double VELOCITY_MAXI = 6553.4;

    public static volatile double POSITION_KP = 400000.0;
    public static volatile double POSITION_KI = 10000000.0;
    public static volatile double POSITION_KD = 1000.0;
    public static volatile double POSITION_MAXI = 2374.4;
    public static volatile double POSITION_ERROR_THRESHOLD = 20;

    private PIDVASMotorController motorVelocityController = null;

    public static volatile double MIN_MOTOR_RPM = 0.0;
    public static volatile double MAX_MOTOR_RPM = 350.0;

    private LowHighPassLimiter separatorMotorVelocityLimiter = null;

    private PIDVASMotorController motorPositionController;

    private double previousSeparatorMotorTarget = 0;
    private int separatorMotorEncoderValue = 0;

    private double separatorMotorTarget = 0;
    private final ElapsedTime separatorMotorStallDetectionTimer = new ElapsedTime();

    private ColorSensor puckColorSensor;

    private int teamPucksCollected = 0;
    private int opponentPucksCollected = 0;

    @ColorInt
    private int lastReadColorInt = Color.BLACK;

    private FieldColor lastReadColor = FieldColor.WHITE;

    private FieldColor lastCollectedPuckColor = FieldColor.WHITE;
    private final ElapsedTime timeSinceLastPuckCollection = new ElapsedTime();

    public static volatile int RED_PUCK_COLOR_R = 148;
    public static volatile int RED_PUCK_COLOR_G = 25;
    public static volatile int RED_PUCK_COLOR_B = 1;
    public static volatile int BLUE_PUCK_COLOR_R = 11;
    public static volatile int BLUE_PUCK_COLOR_G = 45;
    public static volatile int BLUE_PUCK_COLOR_B = 120;
    public static volatile int NO_PUCK_COLOR_R = 110;
    public static volatile int NO_PUCK_COLOR_G = 128;
    public static volatile int NO_PUCK_COLOR_B = 108;

    private final ColorReference puckColorSensorReference = new ColorReferenceMap(
            new EnumMap<FieldColor, Integer>(FieldColor.class) {{
                put(FieldColor.RED, rgb(RED_PUCK_COLOR_R, RED_PUCK_COLOR_G, RED_PUCK_COLOR_B));
                put(FieldColor.BLUE, rgb(BLUE_PUCK_COLOR_R, BLUE_PUCK_COLOR_G, BLUE_PUCK_COLOR_B));
                put(FieldColor.WHITE, rgb(NO_PUCK_COLOR_R, NO_PUCK_COLOR_G, NO_PUCK_COLOR_B));
            }});

    public Separator(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        puckColorSensor = robot.hardware.puckColorSensor;
        puckColorSensor.enableLed(true);

        separatorMotor = robot.hardware.separatorMotor;
        separatorMotor.setDirection(MOTOR_DIRECTION);
        separatorMotor.setZeroPowerBehavior(MOTOR_ZEROPOWERBEHAVIOR);

        motorVelocityController = new PIDVASMotorController(separatorMotorPowerLimiter::update, separatorMotor::getVelocity,
                () -> VELOCITY_KP, () -> VELOCITY_KI, () -> VELOCITY_KD, () -> VELOCITY_KV, () -> 0, () -> VELOCITY_KS, () -> VELOCITY_MAXI, 0, false);

        separatorMotorVelocityLimiter = new LowHighPassLimiter(motorVelocityController::update,
                MIN_MOTOR_RPM * MOTOR_RPM_TO_TPS_RATIO,
                MAX_MOTOR_RPM * MOTOR_RPM_TO_TPS_RATIO);

        motorPositionController = new PIDVASMotorController(separatorMotorVelocityLimiter::update, separatorMotor::getCurrentPosition,
                () -> POSITION_KP, () -> POSITION_KI, () -> POSITION_KD, () -> 0, () -> 0, () -> 0, () -> POSITION_MAXI, POSITION_ERROR_THRESHOLD, false);

        separatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        separatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        if ((!robot.fieldSensor.isOnTeamSquare()) && motorPositionController.isAtTarget()) {
            lastReadColorInt = puckColorSensor.argb();
            lastReadColor = puckColorSensorReference.matchClosestColor(lastReadColorInt);
            FieldColor teamColor = robot.fieldSensor.getTeamFieldColor();
            if (lastReadColor == teamColor) {
                setSeparatorMotorTarget(separatorMotorTarget + MOTOR_ROTATION_ENCODER_TICKS);
                teamPucksCollected++;
                lastCollectedPuckColor = lastReadColor;
                timeSinceLastPuckCollection.reset();
            } else if (lastReadColor == teamColor.opposite()) {
                setSeparatorMotorTarget(separatorMotorTarget - MOTOR_ROTATION_ENCODER_TICKS);
                opponentPucksCollected++;
                lastCollectedPuckColor = lastReadColor;
                timeSinceLastPuckCollection.reset();
            }
        }
        if (separatorMotorStallDetectionTimer.seconds() > MOTOR_STALL_DETECTION_S) {
            setSeparatorMotorTarget(previousSeparatorMotorTarget);
            if (separatorMotorTarget > previousSeparatorMotorTarget)
                teamPucksCollected--;
            else
                opponentPucksCollected--;
        }
        if (!doubleEquals(motorPositionController.getTarget(), separatorMotorTarget) || motorPositionController.isAtTarget()) {
            separatorMotorStallDetectionTimer.reset();
        }

        separatorMotorEncoderValue = separatorMotor.getCurrentPosition();
        motorPositionController.update(separatorMotorTarget);
    }

    public int getSeparatorMotorEncoderValue(){
        return separatorMotorEncoderValue;
    }

    public double getSeparatorMotorTarget() {
        return separatorMotorTarget;
    }

    private void setSeparatorMotorTarget(double newSeparatorMotorTarget) {
        this.previousSeparatorMotorTarget = this.separatorMotorTarget;
        this.separatorMotorTarget = newSeparatorMotorTarget;
    }

    public int getLastReadColorInt() {
        return lastReadColorInt;
    }

    public FieldColor getLastReadColor() {
        return lastReadColor;
    }

    public FieldColor getLastCollectedPuckColor() {
        return lastCollectedPuckColor;
    }

    public int getTeamPucksCollected() {
        return teamPucksCollected;
    }

    public int getOpponentPucksCollected() {
        return opponentPucksCollected;
    }

    public double getSecondsSinceLastPuckCollection() {
        return timeSinceLastPuckCollection.seconds();
    }

}
