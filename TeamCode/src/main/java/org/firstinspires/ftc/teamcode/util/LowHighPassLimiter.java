package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import java.util.function.DoubleConsumer;

public class LowHighPassLimiter {

    private final DoubleConsumer consumer;
    private final double lowerBound;
    private final double higherBound;

    public LowHighPassLimiter(DoubleConsumer consumer, double lowerBound, double higherBound) {
        this.consumer = consumer;
        this.lowerBound = lowerBound;
        this.higherBound = higherBound;
    }

    public void update(double value) {
        consumer.accept(
                (value) > higherBound ?
                        higherBound * signum(value) :
                        abs(value) < lowerBound ?
                                (abs(value) < lowerBound / 2 ? 0 : lowerBound * 0.5) :
                                value);
    }
}
