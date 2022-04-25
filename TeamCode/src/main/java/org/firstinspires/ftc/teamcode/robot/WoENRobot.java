package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.stream.Stream;

public class WoENRobot {

    public final Separator separator = new Separator(this);
    public final Drivetrain drivetrain = new Drivetrain(this);
    public final LedStrip ledStrip = new LedStrip(this);
    public final Odometry odometry = new Odometry(this);
    public final Barrier barrier = new Barrier(this);
    protected final Hardware hardware;
    protected final LinearOpMode opMode;
    private final Stream<LoopedSubsystem> allModules = Stream.of(
            separator,
            drivetrain,
            ledStrip,
            odometry,
            barrier
    );

    public WoENRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardware = new Hardware(opMode.hardwareMap);
    }

    public void initialize() {
        allModules.forEach(LoopedSubsystem::initialize);
    }

    public void update() {
        allModules.forEach(LoopedSubsystem::update);
    }
}
