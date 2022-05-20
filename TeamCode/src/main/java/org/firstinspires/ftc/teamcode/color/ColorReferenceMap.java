package org.firstinspires.ftc.teamcode.color;

import static org.firstinspires.ftc.teamcode.color.ColorUtils.rgbDistance;

import androidx.annotation.ColorInt;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.stream.Collectors;

public class ColorReferenceMap implements ColorReference {

    private final EnumMap<FieldColor, Integer> referenceMap;

    public ColorReferenceMap(EnumMap<FieldColor, Integer> referenceMap) {
        this.referenceMap = referenceMap;
    }

    public FieldColor matchClosestColor(@ColorInt int matchColor) {
        return Collections.min(referenceMap.entrySet().stream()
                .collect(Collectors.toMap(Map.Entry::getKey, colorReferenceEntry -> rgbDistance(colorReferenceEntry.getValue(), matchColor)))
                .entrySet(), Map.Entry.comparingByValue()).getKey();
    }

    public void put(FieldColor fieldColor, @ColorInt int colorData) {
        referenceMap.put(fieldColor, colorData);
    }
}
