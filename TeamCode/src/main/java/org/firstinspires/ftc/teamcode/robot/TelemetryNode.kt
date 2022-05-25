package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.function.Consumer
import java.util.function.Supplier

@Config
class TelemetryNode(robot: WoENRobot) : RobotModule(robot) {

    var telemetry: Telemetry? = null

    override fun initialize() {
        telemetry = when (TELEMETRY_TYPE) {
            TelemetryType.OFF -> null
            TelemetryType.DASHBOARD -> FtcDashboard.getInstance().telemetry
            TelemetryType.DRIVER_STATION -> robot.opMode.telemetry
            TelemetryType.DUAL -> MultipleTelemetry(
                robot.opMode.telemetry,
                FtcDashboard.getInstance().telemetry
            )
        }
        telemetry?.let { it.msTransmissionInterval = TRANSMISSION_INTERVAL_MS }
    }

    val executorSupplier = Supplier { Executors.newSingleThreadExecutor() }
    var executor: ExecutorService = executorSupplier.get()
    val refreshTimer = ElapsedTime()
    var telemetryCallback: Consumer<Telemetry>? = null
    val telemetryRunnable = Runnable {
        telemetry?.let { telemetry ->
            telemetryCallback?.accept(telemetry)
            telemetry.addData("Runtime", robot.opMode.runtime)
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

    enum class TelemetryType {
        OFF, DASHBOARD, DRIVER_STATION, DUAL
    }

    companion object {
        @Volatile @JvmField var TELEMETRY_TYPE = TelemetryType.DRIVER_STATION
        @Volatile @JvmField var TRANSMISSION_INTERVAL_MS = 50;
    }
}