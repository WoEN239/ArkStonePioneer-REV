package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.WoENRobot;

@TeleOp
public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        WoENRobot robot = new WoENRobot(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot.initialize();

        telemetry.addData("Status", "Initialized, ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
