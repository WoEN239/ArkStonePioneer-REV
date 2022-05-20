package org.firstinspires.ftc.teamcode.color;

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
        return Collections.min(referenceSet.stream().map(
                        colorReferenceEntry -> Pair.create(colorReferenceEntry.first, rgbDistance(colorReferenceEntry.second, matchColor)))
                .collect(Collectors.toSet()), Comparator.comparing(pair -> pair.second)).first;
    }

    public void put(FieldColor fieldColor, @ColorInt int colorData) {
        referenceSet.add(Pair.create(fieldColor, colorData));
    }
}
