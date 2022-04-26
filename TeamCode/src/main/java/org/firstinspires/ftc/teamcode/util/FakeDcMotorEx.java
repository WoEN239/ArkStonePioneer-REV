package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class FakeDcMotorEx implements DcMotorEx {

    MotorConfigurationType motorType = MotorConfigurationType.getUnspecifiedMotorType();

    @Override
    public MotorConfigurationType getMotorType() {
        return motorType;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        this.motorType = motorType;
    }

    FakeDcMotorController fakeDcMotorController = new FakeDcMotorController();

    @Override
    public DcMotorController getController() {
        return fakeDcMotorController;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.UNKNOWN;

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    @Override
    public void setPowerFloat() {

    }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int position) {

    }

    @Override
    public int getTargetPosition() {
        return 0;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }

    RunMode mode = RunMode.RUN_WITHOUT_ENCODER;

    @Override
    public void setMode(RunMode mode) {
        this.mode = mode;
    }

    @Override
    public RunMode getMode() {
        return mode;
    }

    @Override
    public void setMotorEnable() {

    }

    @Override
    public void setMotorDisable() {

    }

    @Override
    public boolean isMotorEnabled() {
        return true;
    }

    @Override
    public void setVelocity(double angularRate) {

    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {

    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return 0;
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {

    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {

    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {

    }

    @Override
    public void setPositionPIDFCoefficients(double p) {

    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return new PIDCoefficients(0, 0, 0);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return new PIDFCoefficients(0, 0, 0, 0);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {

    }

    @Override
    public int getTargetPositionTolerance() {
        return 0;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    private Direction direction;

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPower(double power) {

    }

    @Override
    public double getPower() {
        return 0;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "Fake DC Motor";
    }

    @Override
    public String getConnectionInfo() {
        return "Connected";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    private class FakeDcMotorController implements DcMotorController {

        @Override
        public void setMotorType(int motor, MotorConfigurationType motorType) {

        }

        @Override
        public MotorConfigurationType getMotorType(int motor) {
            return motorType;
        }

        @Override
        public void setMotorMode(int motor, RunMode mode) {

        }

        @Override
        public RunMode getMotorMode(int motor) {
            return mode;
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
        public void setMotorZeroPowerBehavior(int motor, ZeroPowerBehavior zeroPowerBehavior) {

        }

        @Override
        public ZeroPowerBehavior getMotorZeroPowerBehavior(int motor) {
            return zeroPowerBehavior;
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
            return Manufacturer.Unknown;
        }

        @Override
        public String getDeviceName() {
            return "Fake DC motor controller";
        }

        @Override
        public String getConnectionInfo() {
            return "Connected";
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
}
