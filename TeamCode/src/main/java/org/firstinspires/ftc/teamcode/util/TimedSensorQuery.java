package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TimedSensorQuery<T> {

    private final ElapsedTime queryTimer = new ElapsedTime();
    private Supplier<T> sensorValueSupplier = null;
    private T lastValue = null;
    private final double updateTimeSeconds;

    public TimedSensorQuery(Supplier<T> sensorValueSupplier, double refreshRateHz) {
        this.sensorValueSupplier = sensorValueSupplier;
        updateTimeSeconds = 1 / refreshRateHz;
    }

    public TimedSensorQuery(Supplier<T> sensorValueSupplier) {
        this(sensorValueSupplier, 100);
    }

    public T getValue() {
        if (lastValue == null) lastValue = sensorValueSupplier.get();
        else if (queryTimer.seconds() > updateTimeSeconds) {
            lastValue = sensorValueSupplier.get();
            queryTimer.reset();
        }
        return lastValue;
    }
}
