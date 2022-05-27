package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.superclasses.RobotModule;
import org.firstinspires.ftc.teamcode.util.TimedSender;

@Config
public class Barrier extends RobotModule {

    private Servo barrierServo;

    public static volatile double SERVO_REFRESH_RATE_HZ = 0.25;

    private final TimedSender<Double> barrierServoPositionSender = new TimedSender<>(position -> barrierServo.setPosition(position), SERVO_REFRESH_RATE_HZ);

    public static volatile double BARRIER_OPEN = 0.36;
    public static volatile double BARRIER_CLOSE = 0.47;

    public static boolean USE_ANGLE = true;

    public static volatile double MIN_OPEN_ANGLE = toRadians(-15f);
    public static volatile double MAX_OPEN_ANGLE = toRadians(105f);

    private final ElapsedTime timeSinceLastBarrierOpening = new ElapsedTime();

    public Barrier(WoENRobot robot) {
        super(robot);
    }

    @Override
    public void initialize() {
        barrierServo = robot.hardware.barrierServo;
        //if (barrierServo instanceof PwmControl)
        //    ((PwmControl) barrierServo).setPwmEnable();
    }

    @Override
    public void update() {
        boolean open = robot.fieldSensor.isOnTeamSquare() && (!USE_ANGLE ||
                (MIN_OPEN_ANGLE < robot.orientationSensor.getOrientation() && robot.orientationSensor.getOrientation() < MAX_OPEN_ANGLE));
        barrierServoPositionSender.trySend(open ? BARRIER_OPEN : BARRIER_CLOSE);
        if (open)
            timeSinceLastBarrierOpening.reset();
    }

    public double getSecondsSinceLastBarrierOpening() {
        return timeSinceLastBarrierOpening.seconds();
    }
}
