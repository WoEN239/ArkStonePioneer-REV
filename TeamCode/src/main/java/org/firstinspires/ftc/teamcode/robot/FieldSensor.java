package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.color.ColorReference;
import org.firstinspires.ftc.teamcode.color.FieldColor;

public class FieldSensor extends RobotModule {

    public static boolean ADJUST_REFERENCE_ON_INITIALISATION = true;
    public static boolean PICK_COLOR_DYNAMICALLY = true;
    public static FieldColor PRESELECTED_TEAM_COLOR = FieldColor.RED;
    public static double COLOR_DETECTION_TIME_WINDOW_S = .15;

    private ColorSensor fieldColorSensor;

    @ColorInt
    private int lastReadColorInt = Color.BLACK;

    private FieldColor teamFieldColor = FieldColor.WHITE;
    private FieldColor detectedColor = FieldColor.WHITE;
    private FieldColor lastReadColor = FieldColor.WHITE;

    private ColorReference fieldColorSensorReference = ColorReference.defaultColorReference();

    private ElapsedTime timeSinceLastColorChange = new ElapsedTime();

    public FieldSensor(WoENRobot robot) {
        super(robot);
    }

    public void initialize() {
        fieldColorSensor = robot.hardware.fieldColorSensor;
        if (PICK_COLOR_DYNAMICALLY)
            pickColor();
        else
            teamFieldColor = PRESELECTED_TEAM_COLOR;
    }

    private void pickColor() {
        lastReadColorInt = fieldColorSensor.argb();
        teamFieldColor = fieldColorSensorReference.matchClosestColor(lastReadColorInt);
        if (ADJUST_REFERENCE_ON_INITIALISATION)
            fieldColorSensorReference.put(teamFieldColor, lastReadColorInt);
    }

    @Override
    public void update() {
        lastReadColorInt = fieldColorSensor.argb();
        FieldColor newReadColor = fieldColorSensorReference.matchClosestColor(lastReadColorInt);
        if (newReadColor != lastReadColor)
            timeSinceLastColorChange.reset();
        else if (timeSinceLastColorChange.seconds() > COLOR_DETECTION_TIME_WINDOW_S)
            detectedColor = newReadColor;
        lastReadColor = newReadColor;
    }

    public FieldColor getTeamFieldColor() {
        return teamFieldColor;
    }

    @ColorInt
    public int getLastReadColorInt() {
        return lastReadColorInt;
    }

    public FieldColor getDetectedColor() {
        return detectedColor;
    }

    public boolean isOnTeamSquare() {
        return detectedColor == teamFieldColor;
    }

    public boolean isOnOpponentSquare() {
        return detectedColor == teamFieldColor.opposite();
    }


}
