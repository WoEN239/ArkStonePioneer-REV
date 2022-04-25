package org.firstinspires.ftc.robotcore.internal.opmode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.robotcore.internal.hardware.android.Rev3328;

public class ButtonOpModeStarter {

    static Thread starterThread = new Thread();

    public <T extends OpMode> ButtonOpModeStarter(String opModeTransition, OpModeManagerImpl opModeManager) {
        final Blinker blinker;
        {
            Blinker tempBlinker;
            try {
                tempBlinker = opModeManager.getHardwareMap().get(Blinker.class, "Control Hub");
            } catch (IllegalArgumentException e) {
                tempBlinker = null;
            }
            blinker = tempBlinker;
        }
        if (opModeTransition != null && !starterThread.isAlive()) {
            RegisteredOpModes registeredOpModes = RegisteredOpModes.getInstance();
            if (registeredOpModes.getOpMode(opModeTransition) != null) {
                opModeManager.initActiveOpMode(opModeTransition);
                if (blinker != null)
                    blinker.setConstant(Color.YELLOW);
                starterThread = new Thread(() -> {
                    try {
                        boolean started = false;
                        Thread.sleep(1000);
                        while (!opModeManager.getActiveOpModeName().equals(OpModeManagerImpl.DEFAULT_OP_MODE_NAME)) {
                            if (Rev3328.getInstance().getUserButtonPin().getState())
                                if (!started) {
                                    if (blinker != null)
                                        blinker.setConstant(Color.RED);
                                    opModeManager.startActiveOpMode();
                                    Thread.sleep(500);
                                    started = true;
                                } else {
                                    if (blinker != null)
                                        blinker.setConstant(Color.DKGRAY);
                                    opModeManager.stopActiveOpMode();
                                    Thread.sleep(2000);
                                }
                        }
                    } catch (Exception e) {
                    }
                });
                starterThread.start();
            }
        }
    }
}
