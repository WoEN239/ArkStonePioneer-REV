package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class RefreshRateAnalyzer implements LoopedSubsystem {

    ElapsedTime loopTimer = new ElapsedTime();

    private static volatile double ANALYZE_TIME_SECONDS = 1.0;

    private int counter;

    public double getUpdateRateHz() {
        return updateRateHz;
    }

    private double updateRateHz;

    @Override
    public void initialize() {
        counter = 0;
        updateRateHz = 0;
        loopTimer.reset();
    }

    @Override
    public void update() {
        if (loopTimer.seconds() >= ANALYZE_TIME_SECONDS) {
            updateRateHz = (double) counter / ANALYZE_TIME_SECONDS;
            counter = 0;
            loopTimer.reset();
        } else
            counter++;
    }
}
