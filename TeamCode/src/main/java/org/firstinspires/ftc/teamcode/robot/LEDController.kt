package org.firstinspires.ftc.teamcode.robot

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.color.FieldColor
import org.firstinspires.ftc.teamcode.robot.superclasses.RobotModule
import org.firstinspires.ftc.teamcode.util.MathUtils.getSineWave
import org.firstinspires.ftc.teamcode.util.TimedSender

@Config
class LEDController(robot: WoENRobot?) : RobotModule(robot) {
    private val refreshTimer = ElapsedTime()
    var state: State = State.OFF
    private lateinit var ledStrip: DcMotorSimple
    private lateinit var rgbFanControl: RgbFanControl
    private var controlHubLEDSender: TimedSender<Int> = TimedSender({
        robot?.hardware?.controlHubLED?.setConstant(it)
    }, 0.25)
    private var expansionHubLEDSender: TimedSender<Int> = TimedSender({
        robot?.hardware?.expansionHubLED?.setConstant(it)
    }, 0.25)
    private var ledStripPowerSender = TimedSender<Double> { ledStrip.power }

    override fun initialize() {
        ledStrip = robot.hardware.ledStrip
        ledStrip.direction = DcMotorSimple.Direction.FORWARD
        rgbFanControl = RgbFanControl(robot.hardware.rgbFan)
    }

    override fun update() {
        when (state) {
            State.COLOR_INDICATOR -> {
                if (refreshTimer.seconds() > 1.0 / REFRESH_RATE_HZ) {
                    controlHubLEDSender.trySend(robot.fieldSensor.lastReadColorInt)
                    val puckCollectedRecently =
                        robot.separator.secondsSinceLastPuckCollection < PUCK_INDICATION_S
                    expansionHubLEDSender.trySend(if (puckCollectedRecently) robot.separator.lastCollectedPuckColor.toColorInt() else Color.DKGRAY)
                    if (puckCollectedRecently) rgbFanControl.setColor(
                        RgbFanMode.fromFieldColor(robot.separator.lastCollectedPuckColor),
                        1.0
                    ) else rgbFanControl.setColor(
                        RgbFanMode.fromFieldColor(robot.fieldSensor.teamFieldColor), getSineWave(
                            LED_STRIP_BREATHE_FREQUENCY
                        ) * 0.5 + 0.25
                    )
                    ledStripPowerSender.trySend(getSineWave(LED_STRIP_BREATHE_FREQUENCY))
                    refreshTimer.reset()
                }
            }
            State.WHITE -> {
                rgbFanControl.setColor(RgbFanMode.WHITE)
                ledStripPowerSender.trySend(1.0)
            }
            State.OFF -> {
                rgbFanControl.setColor(RgbFanMode.OFF)
                ledStripPowerSender.trySend(.0)
            }
            State.RED -> {
                rgbFanControl.setColor(RgbFanMode.RED, 1.0)
                ledStripPowerSender.trySend(0.5)
            }
            State.BLUE -> {
                rgbFanControl.setColor(RgbFanMode.BLUE, 1.0)
                ledStripPowerSender.trySend(0.5)
            }
        }

    }

    enum class State {
        COLOR_INDICATOR, WHITE, OFF, RED, BLUE
    }

    private class RgbFanControl(motorPort: DcMotor) {
        private val zeroPowerBehaviorSender: TimedSender<ZeroPowerBehavior>
        private val powerSender: TimedSender<Double>
        fun setColor(mode: RgbFanMode, value: Double = 1.0) {
            when (mode) {
                RgbFanMode.RED -> {
                    zeroPowerBehaviorSender.trySend(ZeroPowerBehavior.FLOAT)
                    powerSender.trySend(RGB_FAN_RED_SIGN * value)
                }
                RgbFanMode.BLUE -> {
                    zeroPowerBehaviorSender.trySend(ZeroPowerBehavior.FLOAT)
                    powerSender.trySend(-RGB_FAN_RED_SIGN * value)
                }
                RgbFanMode.WHITE -> {
                    zeroPowerBehaviorSender.trySend(ZeroPowerBehavior.BRAKE)
                    powerSender.trySend(.0)
                }
                RgbFanMode.OFF -> {
                    zeroPowerBehaviorSender.trySend(ZeroPowerBehavior.FLOAT)
                    powerSender.trySend(.0)
                }
            }
        }

        init {
            zeroPowerBehaviorSender = TimedSender({ zeroPowerBehavior: ZeroPowerBehavior? ->
                motorPort.zeroPowerBehavior = zeroPowerBehavior
            }, 0.1)
            powerSender = TimedSender({ power: Double? -> motorPort.power = power!! }, 0.1)
        }
    }

    private enum class RgbFanMode {
        RED, BLUE, WHITE, OFF;

        companion object {
            fun fromFieldColor(fieldColor: FieldColor): RgbFanMode {
                return when (fieldColor) {
                    FieldColor.RED -> RED
                    FieldColor.BLUE -> BLUE
                    FieldColor.WHITE -> WHITE
                }
            }
        }
    }

    companion object {
        @Volatile @JvmField var REFRESH_RATE_HZ = 25.0
        @Volatile @JvmField var PUCK_INDICATION_S = 2.0
        @Volatile @JvmField var LED_STRIP_BREATHE_FREQUENCY = 1.0
        @Volatile @JvmField var RGB_FAN_RED_SIGN = 1
    }
}