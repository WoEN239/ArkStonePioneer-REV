package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.WoENRobot;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        WoENRobot robot = new WoENRobot(this);
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.initialize();
        telemetry.addData("Status", "Initialized, ready to start");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        while (opModeIsActive()) {
            robot.drivetrain.setSpeedFraction(-gamepad1.left_stick_y, gamepad1.right_stick_x);
            robot.update();
        }
    }
}
