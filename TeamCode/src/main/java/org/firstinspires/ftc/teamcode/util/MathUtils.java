package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.abs;

public class MathUtils {
    private static final double EPSILON = 1e-5;

    public static double sqr(double num) {
        return num * num;
    }

    public static boolean doubleEquals(double d1, double d2) {
        return abs(d1 - d2) < EPSILON;
    }
}
