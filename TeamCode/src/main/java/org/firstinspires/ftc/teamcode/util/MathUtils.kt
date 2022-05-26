package org.firstinspires.ftc.teamcode.util

object MathUtils {

    const val EPSILON = 1e-5
    private const val MILLIS_TO_RADIANS = Math.PI * 2.0 * 0.001
    @JvmStatic
    fun sqr(num: Double): Double {
        return num * num
    }

    @JvmStatic
    fun doubleEquals(d1: Double, d2: Double): Boolean {
        return Math.abs(d1 - d2) < EPSILON
    }

    @JvmStatic
    fun getSineWave(frequency: Double): Double {
        return 0.5 - Math.cos(System.currentTimeMillis() * frequency * MILLIS_TO_RADIANS) * 0.5
    }

    fun angleWrap(angle: Double): Double {
        var resultingAngle = angle
        while (angle > Math.PI) resultingAngle -= Math.PI * 2
        while (angle < -Math.PI) resultingAngle += Math.PI * 2
        return resultingAngle
    }

    fun angleWrapHalf(angle1: Double): Double {
        var angle = angle1
        while (angle > Math.PI / 2) angle -= Math.PI
        while (angle < -Math.PI / 2) angle += Math.PI
        return angle
    }
}