package org.firstinspires.ftc.teamcode.robot;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.color.ColorReference;
import org.firstinspires.ftc.teamcode.color.ColorReferenceMap;
import org.firstinspires.ftc.teamcode.color.FieldColor;
import org.firstinspires.ftc.teamcode.robot.superclasses.RobotModule;
import org.firstinspires.ftc.teamcode.util.TimedQuery;

import java.util.EnumMap;

@Config
public class FieldSensor extends RobotModule {

    public static volatile boolean ADJUST_REFERENCE_ON_INITIALISATION = true;
    public static volatile boolean PICK_COLOR_DYNAMICALLY = true;
    public static volatile FieldColor PRESELECTED_TEAM_COLOR = FieldColor.RED;
    public static volatile double COLOR_DETECTION_TIME_WINDOW_S = .5;
    public static volatile double SENSOR_REFRESH_RATE_HZ = 8;

    private ColorSensor fieldColorSensor;

    private final TimedQuery<Integer> fieldColorSensorQuery = new TimedQuery<>(() -> fieldColorSensor.argb(), SENSOR_REFRESH_RATE_HZ);

    @ColorInt
    private int lastReadColorInt = Color.BLACK;

    private FieldColor teamFieldColor = FieldColor.WHITE;
    private FieldColor detectedColor = FieldColor.WHITE;
    private FieldColor lastReadColor = FieldColor.WHITE;

    public static volatile int RED_FIELD_COLOR_R = 148;
    public static volatile int RED_FIELD_COLOR_G = 25;
    public static volatile int RED_FIELD_COLOR_B = 1;
    public static volatile int BLUE_FIELD_COLOR_R = 11;
    public static volatile int BLUE_FIELD_COLOR_G = 45;
    public static volatile int BLUE_FIELD_COLOR_B = 120;
    public static volatile int WHITE_FIELD_COLOR_R = 110;
    public static volatile int WHITE_FIELD_COLOR_G = 128;
    public static volatile int WHITE_FIELD_COLOR_B = 108;

    private final ColorReference fieldColorSensorReference = new ColorReferenceMap(
            new EnumMap<FieldColor, Integer>(FieldColor.class) {{
                put(FieldColor.RED, rgb(RED_FIELD_COLOR_R, RED_FIELD_COLOR_G, RED_FIELD_COLOR_B));
                put(FieldColor.BLUE, rgb(BLUE_FIELD_COLOR_R, BLUE_FIELD_COLOR_G, BLUE_FIELD_COLOR_B));
                put(FieldColor.WHITE, rgb(WHITE_FIELD_COLOR_R, WHITE_FIELD_COLOR_G, WHITE_FIELD_COLOR_B));
            }});

    private final ElapsedTime timeSinceLastColorChange = new ElapsedTime();

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
        if (teamFieldColor == FieldColor.WHITE)
            teamFieldColor = PRESELECTED_TEAM_COLOR;
    }

    @Override
    public void update() {
        lastReadColorInt = fieldColorSensorQuery.getValue();
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
