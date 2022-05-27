package org.firstinspires.ftc.teamcode.util

import java.util.*
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

class Pose2D @JvmOverloads constructor(var x: Double = 0.0, var y: Double = 0.0, heading: Double = 0.0) {
    var heading: Double
    operator fun plus(p2: Pose2D): Pose2D {
        return Pose2D(x + p2.x, y + p2.y, heading + p2.heading)
    }

    operator fun times(p2: Pose2D): Pose2D {
        return Pose2D(x * p2.x, y * p2.y, heading * p2.heading)
    }

    operator fun div(p2: Pose2D): Pose2D {
        return Pose2D(x / p2.x, y / p2.y, heading / p2.heading)
    }

    operator fun minus(p2: Pose2D): Pose2D {
        return Pose2D(x - p2.x, y - p2.y, heading - p2.heading)
    }

    operator fun times(d: Double): Pose2D {
        return Pose2D(x * d, y * d, heading * d)
    }

    fun atan(): Double {
        return atan2(y, x)
    }

    fun aCot(): Double {
        return atan2(x, y)
    }

    fun rotatedCW(angle: Double): Pose2D {
        val sinA = sin(angle)
        val cosA = cos(angle)
        return Pose2D(x * cosA + y * sinA, -x * sinA + y * cosA, heading)
    }

    fun rotated(angle: Double): Pose2D {
        val sinA = Math.sin(angle)
        val cosA = Math.cos(angle)
        return Pose2D(x * cosA - y * sinA, x * sinA + y * cosA, heading)
    }

    fun hypot(): Double {
        return hypot(x, y)
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Pose2D) return false
        return MathUtils.doubleEquals(x, other.x) && MathUtils.doubleEquals(
            y,
            other.y
        ) && MathUtils.doubleEquals(heading, other.heading)
    }

    override fun toString(): String {
        return String.format(
            Locale.getDefault(),
            "{x: %.3f, y: %.3f, θ: %.3f}",
            x,
            y,
            Math.toDegrees(heading)
        )
    }

    fun clone(): Pose2D {
        return Pose2D(x, y, heading)
    }

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        result = 31 * result + heading.hashCode()
        return result
    }

    init {
        this.heading = MathUtils.angleWrap(heading)
    }
}