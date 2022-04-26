package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.PI;
import static java.lang.Math.cos;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LEDController extends RobotModule {

    public static double REFRESH_RATE_HZ = 30;
    public static double PUCK_INDICATION_S = 2.0;
    public static double LED_STRIP_BREATHE_PERIOD = 1.0;

    private final ElapsedTime refreshTimer = new ElapsedTime();

    private DcMotorSimple ledStrip;
    private Blinker controlHubLED;
    private Blinker expansionHubLED;

    public LEDController(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        ledStrip = robot.hardware.ledStrip;
        ledStrip.setDirection(DcMotorSimple.Direction.FORWARD);

        controlHubLED = robot.hardware.controlHubLED;
        expansionHubLED = robot.hardware.expansionHubLED;
    }

    private static final double MILLIS_TO_RADIANS = PI * 2.0 * 0.001;

    public double getSineWave(double period) {
        return 0.5 - cos(System.currentTimeMillis() * period * MILLIS_TO_RADIANS) * 0.5;
    }

    @Override
    public void update() {
        if (refreshTimer.seconds() > 1.0 / REFRESH_RATE_HZ) {
            controlHubLED.setConstant(robot.fieldSensor.getLastReadColorInt());
            expansionHubLED.setConstant(robot.separator.getSecondsSinceLastPuckCollection() < PUCK_INDICATION_S ?
                    robot.separator.getLastCollectedPuckColor().toColorInt() : Color.DKGRAY);
            ledStrip.setPower(getSineWave(LED_STRIP_BREATHE_PERIOD));
            refreshTimer.reset();
        }
    }
}
