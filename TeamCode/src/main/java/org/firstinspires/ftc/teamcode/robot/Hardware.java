package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtColorSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.hardware.android.Rev3328;
import org.firstinspires.ftc.teamcode.util.FakeDcMotorEx;

import java.util.function.Supplier;
import java.util.stream.Stream;

public class Hardware {

    protected final DcMotorEx leftBrushMotor;
    protected final DcMotorEx rightBrushMotor;

    protected final DcMotorSimple ledStrip;

    protected final DistanceSensor leftDistanceSensor;
    protected final DistanceSensor rightDistanceSensor;

    protected final DcMotorEx odometer1;
    protected final DcMotorEx odometer2;

    protected final DcMotorEx separatorMotor;
    protected final ColorSensor puckColorSensor;

    protected final ServoEx barrierServo;
    protected final ColorSensor fieldColorSensor;

    protected final DcMotorEx leftMainMotor;
    protected final DcMotorEx rightMainMotor;

    protected final DcMotorEx leftAuxMotor;
    protected final DcMotorEx rightAuxMotor;

    protected final BNO055IMU imu;

    protected final LynxModule controlHub;
    protected final LynxModule expansionHub;

    protected final Supplier<Stream<LynxModule>> allHubs;

    protected final Blinker controlHubLED;
    protected final Blinker expansionHubLED;

    protected final VoltageSensor batteryVoltageSensor;

    protected final DigitalChannel controlHubButton;

    public Hardware(HardwareMap hardwareMap) {
        leftBrushMotor = hardwareMap.get(DcMotorEx.class, "leftBrushMotor");
        rightBrushMotor = hardwareMap.get(DcMotorEx.class, "rightBrushMotor");

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        separatorMotor = hardwareMap.get(DcMotorEx.class, "separatorMotor");
        puckColorSensor = new HiTechnicNxtColorSensor(hardwareMap.get(I2cDeviceSynch.class, "puckColorSensor"));
        //separatorColorSensor = hardwareMap.get(ColorSensor.class, "separatorColorSensor");

        barrierServo = hardwareMap.get(ServoEx.class, "barrierServo");
        fieldColorSensor = new HiTechnicNxtColorSensor(hardwareMap.get(I2cDeviceSynch.class, "fieldColorSensor"));

        leftMainMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMainMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

        leftAuxMotor = hardwareMap.get(DcMotorEx.class, "leftMotorOdometer1");
        rightAuxMotor = hardwareMap.get(DcMotorEx.class, "rightMotorOdometer2");

        odometer1 = hardwareMap.get(DcMotorEx.class, "leftMotorOdometer1");
        odometer2 = hardwareMap.get(DcMotorEx.class, "rightMotorOdometer2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        ledStrip = hardwareMap.get(DcMotorSimple.class, "ledStrip");

        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");

        allHubs = () -> Stream.of(
                controlHub,
                expansionHub
        );

        batteryVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        controlHubLED = hardwareMap.get(Blinker.class, "Control Hub");
        expansionHubLED = hardwareMap.get(Blinker.class, "Expansion Hub 2");

        controlHubButton = Rev3328.getInstance().getUserButtonPin();
    }
}
