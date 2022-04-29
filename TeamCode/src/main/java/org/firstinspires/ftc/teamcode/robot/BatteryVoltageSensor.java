package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.TimedQuery;

@Config
public class BatteryVoltageSensor extends RobotModule {
    public BatteryVoltageSensor(WoENRobot robot) {
        super(robot);
    }

    private VoltageSensor voltageSensor;

    public static double SENSOR_REFRESH_RATE_HZ = 3;

    private final TimedQuery<Double> voltageSensorQuery = new TimedQuery<>(() -> voltageSensor.getVoltage(), SENSOR_REFRESH_RATE_HZ);

    private double voltage = .0;

    private static final double VOLTAGE_REFERENCE = 12.0;

    @Override
    public void initialize() {
        this.voltageSensor = robot.hardware.batteryVoltageSensor;
    }

    public double getKVoltage() {
        if (voltage != 0)
            return VOLTAGE_REFERENCE / voltage;
        else
            return Double.POSITIVE_INFINITY;
    }

    public double getVoltage() {
        return voltage;
    }

    @Override
    public void update() {
        voltage = voltageSensorQuery.getValue();
    }
}
