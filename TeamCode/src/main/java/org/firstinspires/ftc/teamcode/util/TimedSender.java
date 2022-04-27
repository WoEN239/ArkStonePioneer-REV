package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Consumer;

public class TimedSender<T> {

   /* private static final long NANOS_IN_SECOND = 1000000000;

    private final double updateTimeSeconds;
    private final ElapsedTime actionTimer = new ElapsedTime();
    private final Consumer<T> action;
    private T lastSentValue = null;

    public TimedSender(Consumer<T> action, double refreshRateHz) {
        this.action = action;
        updateTimeSeconds = 1 / refreshRateHz;
    }

    public TimedSender(Consumer<T> action) {
        this(action, 50);
    }

    public void trySend(T value) {

        if (actionTimer.seconds() > updateTimeSeconds) {
            action.run();
            actionTimer.reset();
        }
    } */ //TODO
}