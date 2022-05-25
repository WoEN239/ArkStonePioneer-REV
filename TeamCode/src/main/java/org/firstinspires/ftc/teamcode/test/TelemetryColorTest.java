package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

@TeleOp
public class TelemetryColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData(
                    "Team color",
                    ("<font color=\"" + "RED".toLowerCase(Locale.ROOT) + "\">" + "BLUE" + "</font>")
            );
            telemetry.update();
        }
    }
}
