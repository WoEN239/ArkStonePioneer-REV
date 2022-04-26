package org.firstinspires.ftc.teamcode.color;

import android.graphics.Color;

import androidx.annotation.ColorInt;

public enum FieldColor {
    RED, BLUE, WHITE;

    public FieldColor opposite() {
        if (this == RED) return BLUE;
        if (this == BLUE) return RED;
        return WHITE;
    }

    @ColorInt
    public int toColorInt() {
        if (this == RED) return Color.RED;
        if (this == BLUE) return Color.BLUE;
        return Color.WHITE;
    }
}
