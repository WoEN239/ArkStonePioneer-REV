package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.robot.TelemetryNode
import org.firstinspires.ftc.teamcode.robot.WoENRobot
import org.firstinspires.ftc.teamcode.util.SinglePressButton
import java.util.*
import java.util.function.Consumer

abstract class BaseOpMode : LinearOpMode() {
    protected val robot = WoENRobot(this)
    fun execute(actions: Array<Runnable>) {
        Arrays.stream(actions).forEachOrdered { action: Runnable ->
            if (opModeIsActive()) action.run()
            do robot.update() while (!robot.allActionsCompleted() && opModeIsActive())
        }
    }

    var startButtonPresser = SinglePressButton()
    fun startLoop() {
        robot.update()
        if (startButtonPresser.getState(robot.startButton.isPressed)) OpModeManagerImpl.getOpModeManagerOfActivity(
            AppUtil.getInstance().activity
        ).startActiveOpMode()
    }

    override fun waitForStart() {
        while (!isStarted && !isStopRequested) {
            startLoop()
        }
        super.waitForStart()
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot.telemetryNode.telemetry!!.addData("Status", "initializing")
        robot.telemetryNode.telemetry!!.update()
        robot.initialize()
        TelemetryNode.TELEMETRY_TOPIC = TelemetryNode.TelemetryTopic.NONE
        robot.telemetryNode.telemetryCallbacks.add(Consumer { telemetry: Telemetry ->
            telemetry.addData("Status", "initialized")
            telemetry.addLine("<font color=\"green\">Press start to launch program</font>")
        })
        waitForStart()
        TelemetryNode.TELEMETRY_TOPIC = TelemetryNode.TelemetryTopic.COMPETITION_DISPLAY
        main()
        robot.telemetryNode.telemetry!!.addData("Status", "finished")
        robot.telemetryNode.telemetry!!.update()
    }

    abstract fun main()
}