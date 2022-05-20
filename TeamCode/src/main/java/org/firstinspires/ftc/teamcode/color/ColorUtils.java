package org.firstinspires.ftc.teamcode.color;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static org.firstinspires.ftc.teamcode.util.MathUtils.sqr;
import static java.lang.Math.sqrt;

import androidx.annotation.ColorInt;

public class ColorUtils {

    public static double rgbDistance(@ColorInt int color1, @ColorInt int color2) {
        return rgbDistance(red(color1), green(color1), blue(color1), red(color2), green(color2), blue(color2));
    }

    public static double rgbDistance(double r1, double g1, double b1, double r2, double g2, double b2) {
        return sqrt(sqr(r1 - r2) + sqr(g1 - g2) + sqr(b1 - b2));
    }
}
