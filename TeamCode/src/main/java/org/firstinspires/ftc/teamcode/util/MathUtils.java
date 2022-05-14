package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;

public final class MathUtils {

    public static final double EPSILON = 1e-5;

    public static double sqr(double num) {
        return num * num;
    }

    public static boolean doubleEquals(double d1, double d2) {
        return abs(d1 - d2) < EPSILON;
    }


    private static final double MILLIS_TO_RADIANS = PI * 2.0 * 0.001;

    public static double getSineWave(double period) {
        return 0.5 - cos(System.currentTimeMillis() * period * MILLIS_TO_RADIANS) * 0.5;
    }
}
