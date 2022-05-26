package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.TelemetryNode;
import org.firstinspires.ftc.teamcode.robot.WoENRobot;
import org.firstinspires.ftc.teamcode.util.SinglePressButton;

import java.util.Arrays;

public abstract class BaseOpMode extends LinearOpMode {
    protected final WoENRobot robot = new WoENRobot(this);

    Runnable[] actions() {
        return new Runnable[]{};
    }

    public final void execute(Runnable[] runnables) {
        Arrays.stream(runnables).forEachOrdered(action -> {
            if (opModeIsActive()) action.run();
            do robot.update(); while (!robot.movement.getAtTarget() && opModeIsActive());
        });
    }

    SinglePressButton startButtonPresser = new SinglePressButton();

    public void startLoop() {
        robot.update();
        if (startButtonPresser.getState(robot.startButton.isPressed()))
            OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).startActiveOpMode();
    }

    @Override
    public void waitForStart() {
        while (!isStarted() && !isStopRequested()) {
            startLoop();
        }
        super.waitForStart();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.telemetryNode.getTelemetry().addData("Status", "initializing");
        robot.telemetryNode.getTelemetry().update();
        robot.initialize();
        robot.telemetryNode.TELEMETRY_TOPIC = TelemetryNode.TelemetryTopic.NONE;
        robot.telemetryNode.getTelemetryCallbacks().add(telemetry -> {
            telemetry.addData("Status", "initialized");
            telemetry.addLine("<font color=\"green\">Press start to launch program</font>");
        });
        waitForStart();
        robot.telemetryNode.TELEMETRY_TOPIC = TelemetryNode.TelemetryTopic.COMPETITION_DISPLAY;
        execute(actions());
    }
}
