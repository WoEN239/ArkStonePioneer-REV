package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.color.ColorReference;
import org.firstinspires.ftc.teamcode.color.FieldColor;

import java.util.EnumMap;

@Config
public class Separator extends RobotModule {

    public static DcMotorSimple.Direction SEPARATOR_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotor.ZeroPowerBehavior SEPARATOR_MOTOR_ZEROPOWERBEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    public static double SEPARATOR_MOTOR_ROTATION_ENCODER_TICKS = 120 * 2; // Surprisingly LEGO motors have 720 CPR encoders
    public static double SEPARATOR_MOTOR_STALL_DETECTION_S = 0.25;
    @ColorInt
    public static int RED_PUCK_COLOR_INT = Color.RED;
    @ColorInt
    public static int BLUE_PUCK_COLOR_INT = Color.BLUE;
    @ColorInt
    public static int NO_PUCK_COLOR_INT = Color.WHITE;

    private DcMotorEx separatorMotor;
    private double previousSeparatorMotorTarget = 0;

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

    private ColorReference puckColorSensorReference = new ColorReference(
            new EnumMap<FieldColor, Integer>(FieldColor.class) {{
                put(FieldColor.RED, Color.RED);
                put(FieldColor.BLUE, Color.BLUE);
                put(FieldColor.WHITE, Color.WHITE);
            }});

    public Separator(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        puckColorSensor = robot.hardware.puckColorSensor;
        puckColorSensor.enableLed(true);

        separatorMotor = robot.hardware.separatorMotor;
        separatorMotor.setDirection(SEPARATOR_MOTOR_DIRECTION);
        separatorMotor.setZeroPowerBehavior(SEPARATOR_MOTOR_ZEROPOWERBEHAVIOR);
        //separatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        if (robot.fieldSensor.isOnTeamSquare() /* && !separatorMotorController.isAtTarget() */) {
            lastReadColorInt = puckColorSensor.argb();
            lastReadColor = puckColorSensorReference.matchClosestColor(lastReadColorInt);
            FieldColor teamColor = robot.fieldSensor.getTeamFieldColor();
            if (lastReadColor == teamColor) {
                setSeparatorMotorTarget(separatorMotorTarget + SEPARATOR_MOTOR_ROTATION_ENCODER_TICKS);
                teamPucksCollected++;
                lastCollectedPuckColor = lastReadColor;
                timeSinceLastPuckCollection.reset();
            } else if (lastReadColor == teamColor.opposite()) {
                setSeparatorMotorTarget(separatorMotorTarget - SEPARATOR_MOTOR_ROTATION_ENCODER_TICKS);
                opponentPucksCollected++;
                lastCollectedPuckColor = lastReadColor;
                timeSinceLastPuckCollection.reset();
            }
        }
        if (separatorMotorStallDetectionTimer.seconds() > SEPARATOR_MOTOR_STALL_DETECTION_S) {
            setSeparatorMotorTarget(previousSeparatorMotorTarget);
            if (separatorMotorTarget > previousSeparatorMotorTarget)
                teamPucksCollected--;
            else
                opponentPucksCollected--;
        }
        //if (!doubleEquals(separatorMotorController.getTarget(), separatorMotorTarget) || separatorMotorController.isAtTarget())
        {
            separatorMotorStallDetectionTimer.reset();
        }
        /*
        separatorMotorController.setTarget(separatorMotorTarget);
        separatorMotorController.update();
         */
    }

    public double getSeparatorMotorTarget() {
        return separatorMotorTarget;
    }

    private void setSeparatorMotorTarget(double separatorMotorTarget) {
        this.previousSeparatorMotorTarget = this.separatorMotorTarget;
        this.separatorMotorTarget = separatorMotorTarget;
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
