package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.function.Supplier;
import java.util.stream.Stream;

public class WoENRobot {

    public final Separator separator = new Separator(this);
    public final Intake intake = new Intake(this);
    public final DifferentialDrivetrain drivetrain = new DifferentialDrivetrain(this);
    public final DifferentialDrivetrain.DifferentialOdometry odometry = drivetrain.getOdometry();
    public final LEDController LEDController = new LEDController(this);
    public final Barrier barrier = new Barrier(this);
    public final FieldSensor fieldSensor = new FieldSensor(this);
    public final OrientationSensor orientationSensor = new OrientationSensor(this);
    public final WallSensor wallSensor = new WallSensor(this);
    public final BatteryVoltageSensor batteryVoltageSensor = new BatteryVoltageSensor(this);
    public final RefreshRateAnalyzer refreshRateAnalyzer = new RefreshRateAnalyzer();
    public final TelemetryNode telemetryNode = new TelemetryNode(this);
    public final StartButton startButton = new StartButton(this);
    public final Movement movement = new Movement(this);
    protected final Hardware hardware;
    protected final LinearOpMode opMode;
    private final Supplier<Stream<LoopedSubsystem>> allModules = () -> Stream.of(
            separator,
            drivetrain,
            intake,
            LEDController,
            odometry,
            barrier,
            fieldSensor,
            orientationSensor,
            wallSensor,
            batteryVoltageSensor,
            refreshRateAnalyzer,
            telemetryNode,
            movement
    );

    public WoENRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardware = new Hardware(opMode.hardwareMap);
    }

    public void initialize() {
        hardware.controlHubLED.setConstant(Color.rgb(255, 255, 0));
        allModules.get().forEach(LoopedSubsystem::initialize);
        hardware.allHubs.get().forEach(lynxModule -> lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        hardware.controlHubLED.setConstant(Color.rgb(0, 255, 0));
    }

    public void update() {
        hardware.allHubs.get().forEach(LynxModule::clearBulkCache);
        allModules.get().forEachOrdered(LoopedSubsystem::update);
    }
}
