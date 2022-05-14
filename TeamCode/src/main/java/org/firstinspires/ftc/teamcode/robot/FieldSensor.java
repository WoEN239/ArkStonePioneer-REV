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
import org.firstinspires.ftc.teamcode.util.TimedQuery;

import java.util.EnumMap;

@Config
public class FieldSensor extends RobotModule {

    public static boolean ADJUST_REFERENCE_ON_INITIALISATION = true;
    public static boolean PICK_COLOR_DYNAMICALLY = true;
    public static FieldColor PRESELECTED_TEAM_COLOR = FieldColor.RED;
    public static double COLOR_DETECTION_TIME_WINDOW_S = .15;
    public static double SENSOR_REFRESH_RATE_HZ = 5;

    private ColorSensor fieldColorSensor;

    private final TimedQuery<Integer> fieldColorSensorQuery = new TimedQuery<>(() -> fieldColorSensor.argb(), SENSOR_REFRESH_RATE_HZ);

    @ColorInt
    private int lastReadColorInt = Color.BLACK;

    private FieldColor teamFieldColor = FieldColor.WHITE;
    private FieldColor detectedColor = FieldColor.WHITE;
    private FieldColor lastReadColor = FieldColor.WHITE;

    public static int RED_FIELD_COLOR_R = red(Color.RED);
    public static int RED_FIELD_COLOR_G = green(Color.RED);
    public static int RED_FIELD_COLOR_B = blue(Color.RED);
    public static int BLUE_FIELD_COLOR_R = red(Color.BLUE);
    public static int BLUE_FIELD_COLOR_G = green(Color.BLUE);
    public static int BLUE_FIELD_COLOR_B = blue(Color.BLUE);
    public static int WHITE_FIELD_COLOR_R = red(Color.WHITE);
    public static int WHITE_FIELD_COLOR_G = green(Color.WHITE);
    public static int WHITE_FIELD_COLOR_B = blue(Color.WHITE);

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
