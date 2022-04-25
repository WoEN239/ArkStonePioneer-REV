package org.firstinspires.ftc.teamcode.color;

public enum FieldColor {
    RED, BLUE, WHITE;

    public FieldColor opposite() {
        if (this == RED) return BLUE;
        if (this == BLUE) return RED;
        return WHITE;
    }
}
