package org.firstinspires.ftc.teamcode.color;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static org.firstinspires.ftc.teamcode.color.ColorUtils.rgbDistance;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.stream.Collectors;

public class ColorReference {

    private final EnumMap<FieldColor, Integer> colorReferenceMap;

    public ColorReference(EnumMap<FieldColor, Integer> colorReferenceMap) {
        this.colorReferenceMap = colorReferenceMap;
    }

    public static ColorReference defaultColorReference() {
        return new ColorReference(
                new EnumMap<FieldColor, Integer>(FieldColor.class) {{
                    put(FieldColor.RED, Color.RED);
                    put(FieldColor.BLUE, Color.BLUE);
                    put(FieldColor.WHITE, Color.WHITE);
                }});
    }

    public FieldColor matchClosestColor(@ColorInt int matchColor) {
        int matchColorR = red(matchColor);
        int matchColorG = green(matchColor);
        int matchColorB = blue(matchColor);
        EnumMap<FieldColor, Double> radiusMap = new EnumMap(colorReferenceMap.entrySet().stream()
                .collect(Collectors.toMap(Map.Entry::getKey, colorReferenceEntry -> {
                    @ColorInt int colorReference = colorReferenceEntry.getValue();
                    return rgbDistance(red(colorReference), green(colorReference), blue(colorReference), matchColorR, matchColorG, matchColorB);
                })));
        return Collections.min(radiusMap.entrySet(), Map.Entry.comparingByValue()).getKey();
    }

    public void put(FieldColor fieldColor, @ColorInt int adjustedColor) {
        colorReferenceMap.put(fieldColor, adjustedColor);
    }
}
