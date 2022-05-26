package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DigitalChannel

class StartButton(robot: WoENRobot) : RobotModule(robot) {
    private lateinit var button: DigitalChannel
    var isPressed = false
        private set

    override fun initialize() {
        button = robot.hardware.startButton
        button.mode = DigitalChannel.Mode.INPUT
    }

    override fun update() {
        isPressed = button.state
    }
}