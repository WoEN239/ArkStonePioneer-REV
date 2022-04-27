package org.firstinspires.ftc.teamcode.util;

import java.util.function.Supplier;

public class TimedQuery<T> {

    private static final long NANOS_IN_SECOND = 1000000000;

    private long lastUpdateTimeNanos = System.nanoTime();
    private Supplier<T> sensorValueSupplier = null;
    private T lastReceivedValue = null;
    private final long updateTimeNanos;

    public TimedQuery(Supplier<T> sensorValueSupplier, double refreshRateHz) {
        this.sensorValueSupplier = sensorValueSupplier;
        updateTimeNanos = (long)((1.0 / refreshRateHz) * NANOS_IN_SECOND);
    }

    public TimedQuery(Supplier<T> sensorValueSupplier) {
        this(sensorValueSupplier, 100);
    }

    public T getValue() {
        if (lastReceivedValue == null) lastReceivedValue = sensorValueSupplier.get();
        else if (System.nanoTime() - lastUpdateTimeNanos > updateTimeNanos) {
            lastReceivedValue = sensorValueSupplier.get();
            lastUpdateTimeNanos = System.nanoTime();
        }
        return lastReceivedValue;
    }
}
