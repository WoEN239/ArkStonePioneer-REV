package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.util.MathUtils.getSineWave;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.color.FieldColor;
import org.firstinspires.ftc.teamcode.util.TimedSender;

@Config
public class LEDController extends RobotModule {

    public static volatile double REFRESH_RATE_HZ = 25;
    public static volatile double PUCK_INDICATION_S = 2.0;
    public static volatile double LED_STRIP_BREATHE_PERIOD = 1.0;
    public static volatile int RGB_FAN_RED_SIGN = 1;

    private final ElapsedTime refreshTimer = new ElapsedTime();

    private DcMotorSimple ledStrip;
    private RgbFanControl rgbFanControl;

    private TimedSender<Integer> controlHubLEDSender;
    private TimedSender<Integer> expansionHubLEDSender;

    private TimedSender<Double> ledStripPowerSender;

    public LEDController(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        ledStrip = robot.hardware.ledStrip;
        ledStrip.setDirection(DcMotorSimple.Direction.FORWARD);

        rgbFanControl = new RgbFanControl(robot.hardware.rgbFan);

        controlHubLEDSender = new TimedSender<>(robot.hardware.controlHubLED::setConstant, 0.25);
        expansionHubLEDSender = new TimedSender<>(robot.hardware.expansionHubLED::setConstant, 0.25);
        ledStripPowerSender = new TimedSender<>(ledStrip::setPower, 0.25);
    }

    @Override
    public void update() {
        if (refreshTimer.seconds() > 1.0 / REFRESH_RATE_HZ) {
            controlHubLEDSender.trySend(robot.fieldSensor.getLastReadColorInt());
            boolean puckCollectedRecently = robot.separator.getSecondsSinceLastPuckCollection() < PUCK_INDICATION_S;
            expansionHubLEDSender.trySend(puckCollectedRecently ?
                    robot.separator.getLastCollectedPuckColor().toColorInt() : Color.DKGRAY);
            if (puckCollectedRecently)
                rgbFanControl.setColor(RgbFanMode.fromFieldColor(robot.separator.getLastCollectedPuckColor()), 1.0);
            else
                rgbFanControl.setColor(RgbFanMode.fromFieldColor(robot.fieldSensor.getTeamFieldColor()), getSineWave(LED_STRIP_BREATHE_PERIOD) * 0.5 + 0.25);
            ledStripPowerSender.trySend(getSineWave(LED_STRIP_BREATHE_PERIOD));
            refreshTimer.reset();
        }
    }

    private static class RgbFanControl {

        private final TimedSender<DcMotor.ZeroPowerBehavior> zeroPowerBehaviorSender;
        private final TimedSender<Double> powerSender;

        public RgbFanControl(DcMotor motorPort) {
            zeroPowerBehaviorSender = new TimedSender<>(motorPort::setZeroPowerBehavior, 0.1);
            powerSender = new TimedSender<>(motorPort::setPower, 0.1);
        }

        public void setColor(RgbFanMode mode, double value) {
            switch (mode) {
                case RED:
                    zeroPowerBehaviorSender.trySend(DcMotor.ZeroPowerBehavior.FLOAT);
                    powerSender.trySend(RGB_FAN_RED_SIGN * value);
                case BLUE:
                    zeroPowerBehaviorSender.trySend(DcMotor.ZeroPowerBehavior.FLOAT);
                    powerSender.trySend(-RGB_FAN_RED_SIGN * value);
                case WHITE:
                    zeroPowerBehaviorSender.trySend(DcMotor.ZeroPowerBehavior.BRAKE);
                    powerSender.trySend(.0);
                case OFF:
                    zeroPowerBehaviorSender.trySend(DcMotor.ZeroPowerBehavior.FLOAT);
                    powerSender.trySend(.0);
            }
        }

    }

    private enum RgbFanMode {
        RED, BLUE, WHITE, OFF;

        public static RgbFanMode fromFieldColor(FieldColor fieldColor) {
            switch (fieldColor) {
                case RED:
                    return RED;
                case BLUE:
                    return BLUE;
                case WHITE:
                    return WHITE;
            }
            return OFF;
        }
    }
}
