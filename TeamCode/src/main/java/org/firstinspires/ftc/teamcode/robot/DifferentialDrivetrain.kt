package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.robot.superclasses.LoopedSubsystem
import org.firstinspires.ftc.teamcode.robot.superclasses.RobotModule
import org.firstinspires.ftc.teamcode.util.CoupledDcMotorEx
import org.firstinspires.ftc.teamcode.util.MathUtils.angleWrapHalf
import org.firstinspires.ftc.teamcode.util.PIDVASController
import org.firstinspires.ftc.teamcode.util.Pose2D
import org.firstinspires.ftc.teamcode.util.TimedSender
import java.lang.Math.toRadians
import java.util.function.DoubleConsumer
import java.util.function.Supplier
import java.util.stream.Stream
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@Config
class DifferentialDrivetrain(robot: WoENRobot) : RobotModule(robot) {
    private lateinit var leftMotor: DcMotorEx
    val odometry: DifferentialOdometry = DifferentialOdometry()
    private val leftMotorPowerSender =
        TimedSender({ power: Double -> leftMotor.power = power }, MOTOR_REFRESH_RATE_HZ)
    private val leftMotorVoltageCompensator =
        DoubleConsumer { leftMotorPowerSender.trySend(it * robot.batteryVoltageSensor.kVoltage) }
    private val leftMotorController = PIDVASController(
        outputConsumer = { leftMotorVoltageCompensator.accept(it / 32767) },
        sensorValueSupplier = { leftMotor.velocity },
        kP = { VELOCITY_KP },
        kI = { VELOCITY_KI },
        kD = { VELOCITY_KD },
        kV = { VELOCITY_KV },
        kS = { VELOCITY_KS },
        maxI = { VELOCITY_MAXI },
        stopAtTarget = false
    )
    private lateinit var rightMotor: DcMotorEx
    private val rightMotorPowerSender = TimedSender(
        { power: Double -> rightMotor.power = power }, MOTOR_REFRESH_RATE_HZ
    )
    private val rightMotorVoltageCompensator =
        DoubleConsumer { power: Double -> rightMotorPowerSender.trySend(power * robot.batteryVoltageSensor.kVoltage) }
    private val rightMotorController = PIDVASController(
        outputConsumer = { rightMotorVoltageCompensator.accept(it / 32767) },
        sensorValueSupplier = { rightMotor.velocity },
        kP = { VELOCITY_KP },
        kI = { VELOCITY_KI },
        kD = { VELOCITY_KD },
        kV = { VELOCITY_KV },
        kS = { VELOCITY_KS },
        maxI = { VELOCITY_MAXI },
        stopAtTarget = false
    )
    private var forwardTpsTarget = 0.0
    private var turnTpsTarget = 0.0

    fun setSpeedFraction(forwardSpeed: Double, turnSpeed: Double) {
        setSpeed(forwardSpeed * MAX_FORWARD_SPEED_CM, turnSpeed * MAX_ROTATION_SPEED_RAD)
    }

    fun setSpeed(forwardSpeedCm: Double, turnSpeedRadians: Double) {
        forwardTpsTarget = cmToEncoderTicks(forwardSpeedCm)
        turnTpsTarget = rotationRadiansToEncoderTicks(turnSpeedRadians)
    }

    override fun initialize() {
        leftMotor = CoupledDcMotorEx(robot.hardware.leftMainMotor, robot.hardware.leftAuxMotor)
        rightMotor = CoupledDcMotorEx(robot.hardware.rightMainMotor, robot.hardware.rightAuxMotor)
        leftMotor.direction = LEFT_MOTOR_DIRECTION
        rightMotor.direction = RIGHT_MOTOR_DIRECTION
        val allMotors = Supplier { Stream.of(leftMotor, rightMotor) }
        allMotors.get()
            .forEach { dcMotor: DcMotorEx -> dcMotor.zeroPowerBehavior = MOTOR_ZEROPOWERBEHAVIOR }
        allMotors.get()
            .forEach { dcMotor: DcMotorEx -> dcMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER }
        allMotors.get()
            .forEach { dcMotor: DcMotorEx -> dcMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }
    }

    override fun update() {
        if (DRIVETRAIN_CONTROL_MODE == DrivetrainControlMode.FEEDFORWARD) {
            leftMotorVoltageCompensator.accept((forwardTpsTarget - turnTpsTarget) / MAX_MOTOR_TPS)
            rightMotorVoltageCompensator.accept((forwardTpsTarget + turnTpsTarget) / MAX_MOTOR_TPS)
        } else {
            leftMotorController.update(forwardTpsTarget - turnTpsTarget)
            rightMotorController.update(forwardTpsTarget + turnTpsTarget)
        }
    }

    inner class DifferentialOdometry :
        LoopedSubsystem {

        var currentPosition = Pose2D()

        private var previousLeftEncoder = 0.0
        private var previousRightEncoder = 0.0
        private var previousHeading = 0.0

        override fun update() {
            val currentLeftEncoder = leftMotor.currentPosition
            val currentRightEncoder = rightMotor.currentPosition
            val deltaLeftEncoder = currentLeftEncoder - previousLeftEncoder
            val deltaRightEncoder = currentRightEncoder - previousRightEncoder
            val deltaHeadingEncoder =
                toRadians(motorEncodersToRotation(deltaLeftEncoder, deltaRightEncoder))
            val deltaHeading = when (ANGLE_MEASUREMENT_METHOD) {
                AngleMeasurementMethod.ENCODERS -> deltaHeadingEncoder
                AngleMeasurementMethod.IMU -> angleWrapHalf(robot.orientationSensor.orientation - previousHeading)
                AngleMeasurementMethod.HYBRID -> if (robot.orientationSensor.isNewDataAvailable)
                    angleWrapHalf(robot.orientationSensor.orientation - previousHeading)
                else
                    deltaHeadingEncoder
            }
            var deltaPosition = Pose2D(
                motorEncodersToDistance(deltaLeftEncoder, deltaRightEncoder),
                .0,
                deltaHeading
            )
            if (deltaHeadingEncoder != 0.0) {
                val arcAngle = deltaHeadingEncoder * 2.0
                val arcRadius = abs(deltaPosition.x) / arcAngle
                deltaPosition =
                    Pose2D(arcRadius * (1 - cos(arcAngle)), arcRadius * sin(arcAngle), deltaHeading)
            }
            currentPosition += deltaPosition.rotated(currentPosition.heading)
            previousRightEncoder = currentRightEncoder.toDouble()
            previousLeftEncoder = currentLeftEncoder.toDouble()
            previousHeading = currentPosition.heading
        }
    }

    enum class DrivetrainControlMode {
        FEEDFORWARD, PIDVAS
    }

    enum class AngleMeasurementMethod {
        ENCODERS, IMU, HYBRID
    }


    companion object {
        @Volatile @JvmField var LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD
        @Volatile @JvmField var RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE
        @Volatile @JvmField var MOTOR_ZEROPOWERBEHAVIOR = ZeroPowerBehavior.FLOAT
        @Volatile @JvmField var MOTOR_REFRESH_RATE_HZ = 0.5
        @Volatile @JvmField var DRIVETRAIN_CONTROL_MODE = DrivetrainControlMode.FEEDFORWARD
        @Volatile @JvmField var VELOCITY_KP = 27.0
        @Volatile @JvmField var VELOCITY_KI = 0.5
        @Volatile @JvmField var VELOCITY_KD = .0
        @Volatile @JvmField var VELOCITY_KV = 14.46
        @Volatile @JvmField var VELOCITY_KS = 2400.0
        @Volatile @JvmField var VELOCITY_MAXI = 16536.0
        @Volatile @JvmField var ANGLE_MEASUREMENT_METHOD = AngleMeasurementMethod.HYBRID

        private val MOTOR_CPR_NOGEARBOX = 24
        private val MOTOR_GEARING = 10.0
        private val TRACK_WIDTH_CM = 33.70002
        private val WHEEL_DIAMETER_CM = 10.6
        private val MOTOR_CPR = MOTOR_CPR_NOGEARBOX * MOTOR_GEARING
        private val CM_TO_ENCODER_TICKS_RATIO = WHEEL_DIAMETER_CM * Math.PI / MOTOR_CPR
        private val ROTATION_RADIANS_TO_CM_RATIO = 1.0 / TRACK_WIDTH_CM
        private val MAX_MOTOR_RPM = 6000.0
        private val MAX_MOTOR_TPS = MOTOR_CPR * MAX_MOTOR_RPM / 60.0
        val MAX_FORWARD_SPEED_CM = encoderTicksToCm(MAX_MOTOR_TPS)
        val MAX_ROTATION_SPEED_RAD = encoderTicksToRotationRadians(MAX_MOTOR_TPS)

        private fun encoderTicksToCm(ticks: Double): Double {
            return ticks * CM_TO_ENCODER_TICKS_RATIO
        }

        private fun cmToEncoderTicks(cm: Double): Double {
            return cm / CM_TO_ENCODER_TICKS_RATIO
        }

        private fun rotationRadiansToEncoderTicks(radians: Double): Double {
            return cmToEncoderTicks(radians / ROTATION_RADIANS_TO_CM_RATIO)
        }

        private fun encoderTicksToRotationRadians(ticks: Double): Double {
            return encoderTicksToCm(ticks) * ROTATION_RADIANS_TO_CM_RATIO
        }

        private fun motorEncodersToDistance(leftTicks: Double, rightTicks: Double): Double {
            return encoderTicksToCm((leftTicks + rightTicks) * 0.5)
        }

        private fun motorEncodersToRotation(leftTicks: Double, rightTicks: Double): Double {
            return encoderTicksToRotationRadians((-leftTicks + rightTicks) * 0.5)
        }

    }
}