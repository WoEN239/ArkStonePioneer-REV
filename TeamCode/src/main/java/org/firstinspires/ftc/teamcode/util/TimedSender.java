package org.firstinspires.ftc.teamcode.util;

import java.util.function.Consumer;

public class TimedSender<T> {

    private static final long NANOS_IN_SECOND = 1000000000;

    private long lastUpdateTimeNanos = System.nanoTime();
    private final long updateTimeNanos;
    private final Consumer<T> action;
    private T lastSentValue = null;

    private static final double EPSILON = 1 / 32767.0;

    public TimedSender(Consumer<T> action, double refreshRateHz) {
        this.action = action;
        updateTimeNanos = (long) ((1.0 / refreshRateHz) * NANOS_IN_SECOND);
    }

    public TimedSender(Consumer<T> action) {
        this(action, 50);
    }

    public void trySend(T value) {
        boolean valueIsNew = (value != null) ?
                ((value instanceof Number) && (lastSentValue instanceof Number) ?
                        Math.abs(((Number) value).doubleValue() - ((Number) lastSentValue).doubleValue()) > EPSILON :
                        value.equals(lastSentValue)) :
                null == lastSentValue;
        if (valueIsNew || System.nanoTime() - lastUpdateTimeNanos > updateTimeNanos) {
            action.accept(value);
            lastSentValue = value;
            lastUpdateTimeNanos = System.nanoTime();
        }
    }
}