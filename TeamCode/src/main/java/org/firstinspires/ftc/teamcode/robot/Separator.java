package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.color.ColorReference;
import org.firstinspires.ftc.teamcode.color.FieldColor;

import java.util.EnumMap;

public class Separator extends RobotModule {

    public static DcMotorSimple.Direction SEPARATOR_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static double SEPARATOR_MOTOR_ROTATION_ENCODER_TICKS = 120;
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

    @ColorInt
    private int lastReadColorInt = Color.BLACK;

    private FieldColor lastReadColor = FieldColor.WHITE;

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
            } else if (lastReadColor == teamColor.opposite()) {
                setSeparatorMotorTarget(separatorMotorTarget - SEPARATOR_MOTOR_ROTATION_ENCODER_TICKS);
            }
        }
        if (separatorMotorStallDetectionTimer.seconds() > SEPARATOR_MOTOR_STALL_DETECTION_S)
            setSeparatorMotorTarget(previousSeparatorMotorTarget);
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

}
