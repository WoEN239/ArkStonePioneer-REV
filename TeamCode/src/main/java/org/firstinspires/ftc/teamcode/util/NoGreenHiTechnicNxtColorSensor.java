package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtColorSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class NoGreenHiTechnicNxtColorSensor extends HiTechnicNxtColorSensor {
    public NoGreenHiTechnicNxtColorSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient);
    }

    @Override
    public int green() {
        return 0;
    }
}
