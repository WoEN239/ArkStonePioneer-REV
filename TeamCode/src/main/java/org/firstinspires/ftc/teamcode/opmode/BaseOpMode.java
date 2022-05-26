package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.WoENRobot;

public class BaseOpMode extends LinearOpMode {
    protected final WoENRobot robot = new WoENRobot(this);


    public final void execute(Runnable[] runnables, double timeoutSeconds) {
        for (Runnable action : runnables) {
            ElapsedTime elapsedTime = new ElapsedTime();
            if (opModeIsActive()) action.run();
            do robot.update(); while (!robot.movement.getAtTarget() && opModeIsActive());
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
