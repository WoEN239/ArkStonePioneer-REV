package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.util.Pose2D
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class DifferentialOdometry(robot: WoENRobot) : RobotModule(robot) {
    var currentPosition = Pose2D()
        private set
    private val trackWidth = 30.0
    private val wheelDiameter = 10.5
    private val encoderResolution = 24.0
    private val gearBoxRatio = 20.0
    private val encoderTicksToCmRatio = wheelDiameter * PI / (encoderResolution * gearBoxRatio)
    private val rotationDegreesRatio = (180.0 / ((trackWidth / 2.0) * PI)) * (3600 / (3600 + (128.868 + 115.076) * 0.5))
    private var previousLeftEncoder = 0.0
    private var previousRightEncoder = 0.0
    private var previousHeading = 0.0

    private fun encoderTicksToCm(ticks: Double): Double = ticks * encoderTicksToCmRatio
    private fun encoderTicksToRotationDegrees(ticks: Double): Double = encoderTicksToCm(ticks) * rotationDegreesRatio

    private fun angleWrapHalf(angle1: Double): Double {
        var angle = angle1
        while (angle > Math.PI / 2) angle -= Math.PI
        while (angle < -Math.PI / 2) angle += Math.PI
        return angle
    }

    lateinit var leftMotor: DcMotorEx
    lateinit var rightMotor: DcMotorEx
    private val leftEncoder: Int
        get() = leftMotor.currentPosition
    private val rightEncoder: Int
        get() = rightMotor.currentPosition

    override fun initialize() {

    }

    override fun update() {
        val currentLeftEncoder = leftEncoder
        val currentRightEncoder = rightEncoder
        val currentHeading = Math.toRadians(encoderTicksToRotationDegrees((currentLeftEncoder - currentRightEncoder) * 0.5))
        val deltaCurrentLeftEncoder = currentLeftEncoder - previousLeftEncoder
        val deltaCurrentRightEncoder = currentRightEncoder - previousRightEncoder
        val deltaHeading = angleWrapHalf(currentHeading - previousHeading)
        val deltaHeadingEncoder = Math.toRadians(encoderTicksToRotationDegrees((deltaCurrentLeftEncoder - deltaCurrentRightEncoder) * 0.5))
        var deltaPosition = Pose2D(encoderTicksToCm((deltaCurrentLeftEncoder + deltaCurrentRightEncoder) * 0.5), .0, deltaHeading)
        if (deltaHeadingEncoder != 0.0) {
            val arcAngle = deltaHeadingEncoder * 2.0
            val arcRadius = abs(deltaPosition.x) / arcAngle
            deltaPosition = Pose2D(arcRadius * (1 - cos(arcAngle)), arcRadius * sin(arcAngle), deltaHeading)
        }
        currentPosition += deltaPosition.rotatedCW(currentPosition.heading)
        previousRightEncoder = currentRightEncoder.toDouble()
        previousLeftEncoder = currentLeftEncoder.toDouble()
        previousHeading = currentHeading

    }
}