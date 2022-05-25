package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.color.ColorUtils.colorToString
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.function.Consumer
import java.util.function.Supplier

@Config
class TelemetryNode(robot: WoENRobot) : RobotModule(robot) {

    var telemetry: Telemetry? = null

    override fun initialize() {
        telemetry = when (TELEMETRY_DESTINATION) {
            TelemetryDestination.OFF -> null
            TelemetryDestination.DASHBOARD -> FtcDashboard.getInstance().telemetry
            TelemetryDestination.DRIVER_STATION -> robot.opMode.telemetry
            TelemetryDestination.DUAL -> MultipleTelemetry(
                robot.opMode.telemetry,
                FtcDashboard.getInstance().telemetry
            )
        }
        robot.opMode.telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        telemetry?.let { it.msTransmissionInterval = TRANSMISSION_INTERVAL_MS }
    }

    val executorSupplier = Supplier { Executors.newSingleThreadExecutor() }
    var executor: ExecutorService = executorSupplier.get()
    val refreshTimer = ElapsedTime()
    val telemetryCallbacks: List<Consumer<Telemetry>> = ArrayList()
    val telemetryRunnable = Runnable {
        telemetry?.let { telemetry ->
            telemetryCallbacks.forEach { it.accept(telemetry) }
            when (TELEMETRY_TOPIC) {
                TelemetryTopic.COMPETITION_DISPLAY -> {
                    telemetry.addData("Runtime", "%.1f seconds", robot.opMode.runtime)
                    telemetry.addData(
                        "Update rate",
                        "%.0f Hz",
                        robot.refreshRateAnalyzer.updateRateHz
                    )
                    telemetry.addData(
                        "Team color",
                        ("<font color=\"" + robot.fieldSensor.teamFieldColor.toString()
                            .lowercase() + "\">" + robot.fieldSensor.teamFieldColor + "</font>")
                    )
                    telemetry.addData(
                        "Detected color",
                        ("<font color=\"" + robot.fieldSensor.detectedColor.toString()
                            .lowercase() + "\">" + robot.fieldSensor.detectedColor + "</font>")
                    )
                    telemetry.addData("Team pucks collected", robot.separator.teamPucksCollected)
                    telemetry.addLine(
                        "<font color=\"" + robot.fieldSensor.teamFieldColor.toString()
                            .lowercase() + "\">" + "■".repeat(robot.separator.teamPucksCollected) + "</font>" + " "
                    )
                    telemetry.addData("Opponent pucks collected", robot.separator.teamPucksCollected)
                    telemetry.addLine(
                        "<font color=\"" + robot.fieldSensor.teamFieldColor.opposite().toString()
                            .lowercase() + "\">" + "■".repeat(robot.separator.opponentPucksCollected) + "</font>" + " "
                    )
                }
                TelemetryTopic.SEPARATOR -> {
                    robot.separator.let {
                        telemetry.addData("Last read color", it.lastReadColor)
                        telemetry.addData("Last read color RGB", colorToString(it.lastReadColorInt))
                        telemetry.addData("Last collected puck color", it.lastCollectedPuckColor)
                        telemetry.addData("Team pucks collected", it.teamPucksCollected)
                        telemetry.addData("Opponent pucks collected", it.opponentPucksCollected)
                        telemetry.addData(
                            "Seconds since last puck collection",
                            it.secondsSinceLastPuckCollection
                        )
                        telemetry.addData("Motor target", it.separatorMotorTarget)
                        telemetry.addData("Motor encoder value", it.separatorMotorEncoderValue)
                    }
                }
                TelemetryTopic.FIELD_SENSOR -> {
                    robot.fieldSensor.let {
                        telemetry.addData("Detected color", it.detectedColor)
                        telemetry.addData("Last read color RGB", colorToString(it.lastReadColorInt))
                        telemetry.addData("Team field color", it.teamFieldColor)
                        telemetry.addData("Is on team square", it.isOnTeamSquare)
                        telemetry.addData("Is on opponent square", it.isOnOpponentSquare)
                    }
                }
                TelemetryTopic.WALL_SENSOR -> {
                    robot.wallSensor.let {
                        telemetry.addData("Left distance value", it.leftDistance)
                        telemetry.addData("Right distance value", it.rightDistance)
                        telemetry.addData("Estimated distance", it.distanceM)
                        telemetry.addData("Wall detected", it.isNearWall)
                    }
                }
                TelemetryTopic.ORIENTATION_SENSOR -> {
                    robot.orientationSensor.let {
                        telemetry.addData("Orientation", it.orientation);
                    }

                }
                TelemetryTopic.REFRESH_RATE -> {
                    telemetry.addData("Refresh rate (Hz)", robot.refreshRateAnalyzer.updateRateHz)
                }
            }
            telemetry.update()
        }
    }

    override fun update() {
        telemetry?.let {
            if (it.msTransmissionInterval != TRANSMISSION_INTERVAL_MS)
                it.msTransmissionInterval = TRANSMISSION_INTERVAL_MS
            if (refreshTimer.milliseconds() > TRANSMISSION_INTERVAL_MS) {
                if (executor.isTerminated)
                    executor = executorSupplier.get()
                executor.execute(telemetryRunnable)
                refreshTimer.reset()
            }
        }
    }

    enum class TelemetryTopic {
        COMPETITION_DISPLAY, SEPARATOR, FIELD_SENSOR, WALL_SENSOR, ORIENTATION_SENSOR, REFRESH_RATE
    }

    enum class TelemetryDestination {
        OFF, DASHBOARD, DRIVER_STATION, DUAL
    }

    companion object {
        @Volatile @JvmField var TELEMETRY_TOPIC = TelemetryTopic.COMPETITION_DISPLAY
        @Volatile @JvmField var TELEMETRY_DESTINATION = TelemetryDestination.DUAL
        @Volatile @JvmField var TRANSMISSION_INTERVAL_MS = 50;
    }
}