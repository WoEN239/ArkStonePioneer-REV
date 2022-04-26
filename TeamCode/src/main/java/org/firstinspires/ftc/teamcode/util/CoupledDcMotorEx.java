package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CoupledDcMotorEx extends CoupledDcMotor implements DcMotorEx {

    private final DcMotorEx mainMotor;
    private final DcMotorEx auxMotor;

    public CoupledDcMotorEx(DcMotorEx mainMotor, DcMotorEx auxMotor, boolean flipAuxMotorDirection) {
        super(mainMotor, auxMotor, flipAuxMotorDirection);
        this.mainMotor = mainMotor;
        this.auxMotor = auxMotor;
    }

    public CoupledDcMotorEx(DcMotorEx mainMotor, DcMotorEx auxMotor) {
        super(mainMotor, auxMotor);
        this.mainMotor = mainMotor;
        this.auxMotor = auxMotor;
    }

    @Override
    public void setMotorEnable() {
        mainMotor.setMotorEnable();
        auxMotor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        mainMotor.setMotorDisable();
        auxMotor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return mainMotor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        mainMotor.setVelocity(angularRate, unit);
        auxMotor.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return mainMotor.getVelocity();
    }

    @Override
    public void setVelocity(double angularRate) {
        mainMotor.setVelocity(angularRate);
        auxMotor.setVelocity(angularRate);
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return mainMotor.getVelocity(unit);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        mainMotor.setPIDCoefficients(mode, pidCoefficients);
        auxMotor.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        mainMotor.setPIDFCoefficients(mode, pidfCoefficients);
        auxMotor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        mainMotor.setVelocityPIDFCoefficients(p, i, d, f);
        auxMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        mainMotor.setPositionPIDFCoefficients(p);
        auxMotor.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return mainMotor.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return mainMotor.getPIDFCoefficients(mode);
    }

    @Override
    public int getTargetPositionTolerance() {
        return mainMotor.getTargetPositionTolerance();
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        mainMotor.setTargetPositionTolerance(tolerance);
        auxMotor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return mainMotor.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return mainMotor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        mainMotor.setCurrentAlert(current, unit);
        auxMotor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return mainMotor.isOverCurrent() | auxMotor.isOverCurrent();
    }
}
