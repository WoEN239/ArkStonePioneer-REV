package org.firstinspires.ftc.teamcode.usbserial;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class PrizmDcMotorController implements DcMotorController {

    @Override
    public void setMotorType(int motor, MotorConfigurationType motorType) {

    }

    @Override
    public MotorConfigurationType getMotorType(int motor) {
        return null;
    }

    @Override
    public void setMotorMode(int motor, DcMotor.RunMode mode) {

    }

    @Override
    public DcMotor.RunMode getMotorMode(int motor) {
        return null;
    }

    @Override
    public void setMotorPower(int motor, double power) {

    }

    @Override
    public double getMotorPower(int motor) {
        return 0;
    }

    @Override
    public boolean isBusy(int motor) {
        return false;
    }

    @Override
    public void setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int motor) {
        return null;
    }

    @Override
    public boolean getMotorPowerFloat(int motor) {
        return false;
    }

    @Override
    public void setMotorTargetPosition(int motor, int position) {

    }

    @Override
    public int getMotorTargetPosition(int motor) {
        return 0;
    }

    @Override
    public int getMotorCurrentPosition(int motor) {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode(int motor) {

    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
