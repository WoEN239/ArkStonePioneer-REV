package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

import java.lang.reflect.Field;

public class I2CUtils {

    public static void setBusSpeed(HardwareDevice hardwareDevice, LynxI2cDeviceSynch.BusSpeed busSpeed) {
        I2cDeviceSynchDevice<?> i2cDeviceSynchDevice;
        if (hardwareDevice instanceof I2cDeviceSynchDevice)
            i2cDeviceSynchDevice = (I2cDeviceSynchDevice<?>) hardwareDevice;
        else
            throw new IllegalArgumentException("Specified hardware device is not an I2C device");
        Field deviceClientField;
        try {
            deviceClientField = I2cDeviceSynchDevice.class.getDeclaredField("deviceClient");
        } catch (NoSuchFieldException e) {
            throw new IllegalStateException("Internal error: I2cDeviceSynchDevice has no field \"deviceClient\"");
        }
        deviceClientField.setAccessible(true);
        Object deviceClient;
        try {
            deviceClient = deviceClientField.get(i2cDeviceSynchDevice);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("Can't access \"deviceClient\" field");
        }
        if (deviceClient instanceof LynxI2cDeviceSynch)
            ((LynxI2cDeviceSynch) deviceClient).setBusSpeed(busSpeed);
        else
            throw new IllegalArgumentException("Specified hardware device is not connected to REV hub");
    }

}
