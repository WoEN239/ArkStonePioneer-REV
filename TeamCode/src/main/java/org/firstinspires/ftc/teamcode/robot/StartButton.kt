package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.robot.superclasses.ActionDelegate
import org.firstinspires.ftc.teamcode.robot.superclasses.RobotModule

class StartButton(robot: WoENRobot) : RobotModule(robot), ActionDelegate {
    private lateinit var button: DigitalChannel
    private var state: State = State.NONE

    var isPressed = false
        private set

    fun awaitPress() {
        state = State.AWAITING_PRESS
        actionCompleted = false
    }

    fun awaitRelease() {
        state = State.AWAITING_RELEASE
        actionCompleted = false
    }

    override fun initialize() {
        button = robot.hardware.startButton
        button.mode = DigitalChannel.Mode.INPUT
    }

    override fun update() {
        isPressed = button.state
        actionCompleted = when (state) {
            State.AWAITING_PRESS -> isPressed
            State.AWAITING_RELEASE -> !isPressed
            State.NONE -> true
        }
    }

    enum class State {
        AWAITING_PRESS, AWAITING_RELEASE, NONE
    }

    override var actionCompleted: Boolean = false
        private set
}