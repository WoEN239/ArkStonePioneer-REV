package org.firstinspires.ftc.teamcode.color;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static org.firstinspires.ftc.teamcode.color.ColorUtils.rgbDistance;

import android.util.Pair;

import androidx.annotation.ColorInt;

import java.util.Collections;
import java.util.Comparator;
import java.util.Set;
import java.util.stream.Collectors;

public class ColorReferenceSet implements ColorReference {

    private final Set<Pair<FieldColor, Integer>> referenceSet;

    public ColorReferenceSet(Set<Pair<FieldColor, Integer>> referenceSet) {
        this.referenceSet = referenceSet;
    }

    public FieldColor matchClosestColor(@ColorInt int matchColor) {
        int matchColorR = red(matchColor);
        int matchColorG = green(matchColor);
        int matchColorB = blue(matchColor);
        return Collections.min(referenceSet.stream().map(
                colorReferenceEntry -> {
                    @ColorInt int colorReference = colorReferenceEntry.second;
                    return Pair.create(colorReferenceEntry.first, rgbDistance(red(colorReference), green(colorReference), blue(colorReference), matchColorR, matchColorG, matchColorB));
                }).collect(Collectors.toSet()), Comparator.comparing(pair -> pair.second)).first;
    }

    public void put(FieldColor fieldColor, @ColorInt int colorData) {
        referenceSet.add(Pair.create(fieldColor, colorData));
    }

    /*
    colorReferenceEntry -> {
                    @ColorInt int colorReference = colorReferenceEntry.second
                    return rgbDistance(red(colorReference), green(colorReference), blue(colorReference), matchColorR, matchColorG, matchColorB);
                }
     */
}
