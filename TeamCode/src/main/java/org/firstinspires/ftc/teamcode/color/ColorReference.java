package org.firstinspires.ftc.teamcode.color;

import static org.firstinspires.ftc.teamcode.util.MathUtils.sqr;
import static java.lang.Math.sqrt;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.stream.Collectors;

public class ColorReference {

    private final EnumMap<FieldColor, Integer> colorReferenceMap;

    static final ColorReference defaultColorReference = new ColorReference(
            new EnumMap<FieldColor, Integer>(FieldColor.class) {{
                put(FieldColor.RED, Color.RED);
                put(FieldColor.BLUE, Color.BLUE);
                put(FieldColor.WHITE, Color.WHITE);
            }});

    public ColorReference(EnumMap<FieldColor, Integer> colorReferenceMap) {
        this.colorReferenceMap = colorReferenceMap;
    }

    public FieldColor matchClosestColor(@ColorInt int matchColor) {
        int matchColorR = Color.red(matchColor);
        int matchColorG = Color.green(matchColor);
        int matchColorB = Color.blue(matchColor);
        EnumMap<FieldColor, Double> radiusMap = new EnumMap(colorReferenceMap.entrySet().stream()
                .collect(Collectors.toMap(Map.Entry::getKey, colorReferenceEnry -> {
                    @ColorInt int colorReference = colorReferenceEnry.getValue();
                    @ColorInt int colorRefernceR = Color.red(colorReference);
                    @ColorInt int colorRefernceG = Color.green(colorReference);
                    @ColorInt int colorRefernceB = Color.blue(colorReference);
                    return sqrt(sqr(colorRefernceR - matchColorR) + sqr(colorRefernceG - matchColorG) + sqr(colorRefernceB - matchColorB));
                })));
        return Collections.min(radiusMap.entrySet(), Map.Entry.comparingByValue()).getKey();
    }
}
