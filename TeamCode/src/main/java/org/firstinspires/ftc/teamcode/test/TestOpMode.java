package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.color.ColorUtils;
import org.firstinspires.ftc.teamcode.robot.WoENRobot;

@TeleOp
public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        WoENRobot robot = new WoENRobot(this);

        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.initialize();

        telemetry.addData("Status", "Initialized, ready to start");
        telemetry.update();

        waitForStart();

        robot.telemetryNode.getTelemetryCallbacks().add(telemetry -> {
            telemetry.addData("Status", "Running");
            telemetry.addData("Color", robot.fieldSensor.getDetectedColor());
            telemetry.addData("Team", robot.fieldSensor.getTeamFieldColor());
            telemetry.addData("Value", ColorUtils.colorToString(robot.fieldSensor.getLastReadColorInt()));
            telemetry.addData("Separator color", robot.separator.getLastReadColor());
            telemetry.addData("Hz", robot.refreshRateAnalyzer.getUpdateRateHz());
        });

        while (opModeIsActive()) {
            robot.update();
        }
    }
}
