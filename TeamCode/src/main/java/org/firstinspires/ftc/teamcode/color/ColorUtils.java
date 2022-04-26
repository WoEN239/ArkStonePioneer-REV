package org.firstinspires.ftc.teamcode.color;

import static org.firstinspires.ftc.teamcode.util.MathUtils.sqr;
import static java.lang.Math.sqrt;

public class ColorUtils {
    public static double rgbDistance(double r1, double g1, double b1, double r2, double g2, double b2) {
        return sqrt(sqr(r1 - r2) + sqr(g1 - g2) + sqr(b1 - b2));
    }
}
