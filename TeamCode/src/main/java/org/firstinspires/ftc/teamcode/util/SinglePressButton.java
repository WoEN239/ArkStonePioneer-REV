package org.firstinspires.ftc.teamcode.util;

public class SinglePressButton {
    public boolean lastButtonState;
    private final boolean activate = false;

    public boolean getState(boolean buttonState) {
        boolean result = buttonState && !lastButtonState;
        lastButtonState = buttonState;
        return result;
    }
}
