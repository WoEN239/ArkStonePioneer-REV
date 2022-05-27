package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.superclasses.ActionDelegate
import org.firstinspires.ftc.teamcode.robot.superclasses.RobotModule
import org.firstinspires.ftc.teamcode.util.MathUtils.angleWrap
import org.firstinspires.ftc.teamcode.util.PIDVASController
import org.firstinspires.ftc.teamcode.util.Pose2D
import java.lang.Math.toDegrees
import java.lang.Math.toRadians
import kotlin.math.min

@Config
class Movement(robot: WoENRobot) : RobotModule(robot), ActionDelegate {

    private var angleTarget: Double = .0
    private var distanceTarget: Double = .0
    private var movementStartPoint: Pose2D = Pose2D()
    private val forwardAccelerationTimer = ElapsedTime()
    private val movementTimer = ElapsedTime()

    private fun isTimeout(): Boolean {
        return movementTimer.seconds() > MOVEMENT_TIMEOUT
    }

    private fun getForwardAccelerationSpeedFraction(): Double {
        return min(forwardAccelerationTimer.seconds() / FORWARD_SECONDS_TO_ACCELERATE, 1.0)
    }

    var controlMode = ControlMode.OFF
        private set

    fun stop() {
        controlMode = ControlMode.OFF
        actionCompleted = true
    }

    fun setSpeedManual(forwardSpeed: Double, rotationSpeed: Double) {
        controlMode = ControlMode.MANUAL
        robot.drivetrain.setSpeedFraction(forwardSpeed, rotationSpeed)
    }

    fun move(distanceCm: Double) {
        controlMode = ControlMode.MOVE
        distanceTarget = distanceCm
        movementStartPoint = robot.odometry.currentPosition.clone()
        forwardAccelerationTimer.reset()
        movementTimer.reset()
    }

    fun rotate(rotationDegrees: Double) {
        rotateAbsolute(angleWrap(robot.odometry.currentPosition.heading + toDegrees(rotationDegrees)))
        movementTimer.reset()
    }

    fun rotateAbsolute(rotationDegrees: Double) {
        controlMode = ControlMode.ROTATE
        angleTarget = toDegrees(rotationDegrees)
        movementTimer.reset()
    }

    fun approachWall() {
        controlMode = ControlMode.APPROACH_WALL
        forwardAccelerationTimer.reset()
        movementTimer.reset()
    }

    private val rotationController = PIDVASController(
        kP = { ROTATION_KP },
        kI = { ROTATION_KI },
        kD = { ROTATION_KD },
        maxI = { ROTATION_MAXI },
        stopAtTarget = true,
        errorThreshold = toRadians(ROTATION_ERROR_THRESHOLD)
    )

    private val forwardController = PIDVASController(
        kP = { FORWARD_KP },
        kI = { FORWARD_KI },
        kD = { FORWARD_KD },
        maxI = { FORWARD_MAXI },
        stopAtTarget = true,
        errorThreshold = toRadians(FORWARD_ERROR_THRESHOLD)
    )

    override fun initialize() {
        actionCompleted = false
    }

    override fun update() {
        when (controlMode) {
            ControlMode.MOVE -> {
                robot.drivetrain.setSpeed(
                    forwardController.update((robot.odometry.currentPosition - movementStartPoint).hypot()) *
                            getForwardAccelerationSpeedFraction(),
                    rotationController.update(angleWrap(angleTarget - robot.drivetrain.odometry.currentPosition.heading))
                )
                actionCompleted = forwardController.isAtTarget() || isTimeout()
            }
            ControlMode.ROTATE -> {
                robot.drivetrain.setSpeed(
                    .0,
                    rotationController.update(angleWrap(angleTarget - robot.drivetrain.odometry.currentPosition.heading))
                )
                actionCompleted = rotationController.isAtTarget() || isTimeout()
            }
            ControlMode.APPROACH_WALL -> {
                robot.drivetrain.setSpeed(
                    forwardController.update(robot.wallSensor.distanceM * 100.0, 0.0),
                    rotationController.update(angleWrap(angleTarget - robot.drivetrain.odometry.currentPosition.heading))
                )
                actionCompleted = robot.wallSensor.isNearWall || isTimeout()
            }
            ControlMode.MANUAL ->
                actionCompleted = true
            ControlMode.OFF ->
                robot.drivetrain.setSpeedFraction(.0, .0)
        }
        if (actionCompleted)
            controlMode = ControlMode.OFF
    }

    enum class ControlMode {
        MOVE, ROTATE, APPROACH_WALL, MANUAL, OFF
    }

    override var actionCompleted = false
        private set

    companion object {
        @Volatile @JvmField var MOVEMENT_TIMEOUT = 5.0;
        @Volatile @JvmField var ROTATION_KP = 1.0
        @Volatile @JvmField var ROTATION_KI = 0.0
        @Volatile @JvmField var ROTATION_KD = 0.0
        @Volatile @JvmField var ROTATION_MAXI = DifferentialDrivetrain.MAX_ROTATION_SPEED_RAD
        @Volatile @JvmField var ROTATION_ERROR_THRESHOLD = 5.0
        @Volatile @JvmField var FORWARD_KP = 1.0
        @Volatile @JvmField var FORWARD_KI = 0.0
        @Volatile @JvmField var FORWARD_KD = 0.0
        @Volatile @JvmField var FORWARD_MAXI = DifferentialDrivetrain.MAX_FORWARD_SPEED_CM
        @Volatile @JvmField var FORWARD_ERROR_THRESHOLD = 5.0
        @Volatile @JvmField var FORWARD_SECONDS_TO_ACCELERATE = 0.5
    }
}