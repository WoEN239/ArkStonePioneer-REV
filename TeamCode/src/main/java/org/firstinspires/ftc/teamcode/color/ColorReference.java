package org.firstinspires.ftc.teamcode.color;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import java.util.EnumMap;

public interface ColorReference {

    static ColorReference defaultColorReference() {
        return new ColorReferenceMap(
                new EnumMap<FieldColor, Integer>(FieldColor.class) {{
                    put(FieldColor.RED, Color.RED);
                    put(FieldColor.BLUE, Color.BLUE);
                    put(FieldColor.WHITE, Color.WHITE);
                }});
    }

    FieldColor matchClosestColor(@ColorInt int matchColor);

    void put(FieldColor fieldColor, @ColorInt int colorData);
}
