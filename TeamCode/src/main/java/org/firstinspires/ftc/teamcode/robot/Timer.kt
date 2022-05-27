package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.superclasses.ActionDelegate
import org.firstinspires.ftc.teamcode.robot.superclasses.RobotModule

class Timer(robot: WoENRobot?) : RobotModule(robot), ActionDelegate {
    private val elapsedTime = ElapsedTime()

    var delaySeconds = 0.0
        set(value) {
            elapsedTime.reset()
            field = value
        }

    val timeLeft: Double
        get() = delaySeconds - elapsedTime.seconds()

    override fun initialize() {
        elapsedTime.reset()
    }

    override fun update() {
        if (actionCompleted) delaySeconds = 0.0
    }

    override val actionCompleted: Boolean
        get() = elapsedTime.seconds() > delaySeconds
}