package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class CoupledDcMotor implements DcMotor {

    private final DcMotor mainMotor;
    private final DcMotor auxMotor;

    private final boolean flipAuxMotorDirection;

    public CoupledDcMotor(DcMotor mainMotor, DcMotor auxMotor) {
        this(mainMotor, auxMotor, false);
    }

    public CoupledDcMotor(DcMotor mainMotor, DcMotor auxMotor, boolean flipAuxMotorDirection) {
        this.mainMotor = mainMotor;
        this.auxMotor = auxMotor;
        this.flipAuxMotorDirection = flipAuxMotorDirection;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return mainMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        mainMotor.setMotorType(motorType);
        auxMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return mainMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return mainMotor.getPortNumber();
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return mainMotor.getZeroPowerBehavior();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        mainMotor.setZeroPowerBehavior(zeroPowerBehavior);
        auxMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void setPowerFloat() {
        mainMotor.setPowerFloat();
        auxMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return mainMotor.getPowerFloat();
    }

    @Override
    public int getTargetPosition() {
        return mainMotor.getTargetPosition();
    }

    @Override
    public void setTargetPosition(int position) {
        mainMotor.setTargetPosition(position);
        auxMotor.setTargetPosition(position);
    }

    @Override
    public boolean isBusy() {
        return mainMotor.isBusy() || auxMotor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return mainMotor.getCurrentPosition();
    }

    @Override
    public RunMode getMode() {
        return mainMotor.getMode();
    }

    @Override
    public void setMode(RunMode mode) {
        mainMotor.setMode(mode);
        auxMotor.setMode(mode);
    }

    @Override
    public Direction getDirection() {
        return mainMotor.getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        mainMotor.setDirection(direction);
        auxMotor.setDirection(flipAuxMotorDirection ? direction.inverted() : direction);
    }

    @Override
    public double getPower() {
        return mainMotor.getPower();
    }

    @Override
    public void setPower(double power) {
        mainMotor.setPower(power);
        auxMotor.setPower(power);
    }

    @Override
    public Manufacturer getManufacturer() {
        return mainMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return "Coupled " + mainMotor.getDeviceName() + " and " + auxMotor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return mainMotor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return mainMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        mainMotor.resetDeviceConfigurationForOpMode();
        auxMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        mainMotor.resetDeviceConfigurationForOpMode();
        auxMotor.resetDeviceConfigurationForOpMode();
    }
}
