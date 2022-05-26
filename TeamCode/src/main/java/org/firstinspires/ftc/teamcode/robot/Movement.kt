package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config

@Config
class Movement(robot: WoENRobot) : RobotModule(robot) {

    var atTarget = false
        private set

    override fun initialize() {
        super.initialize()
    }

    override fun update() {
        atTarget = true
    }

    companion object{
        @Volatile @JvmField var MOVEMENT_TIMEOUT = 5.0;
    }

}