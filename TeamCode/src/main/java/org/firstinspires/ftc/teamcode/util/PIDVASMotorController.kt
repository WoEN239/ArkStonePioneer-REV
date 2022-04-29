package org.firstinspires.ftc.teamcode.util

import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.sign

class PIDVASMotorController(
    private val outputConsumer: DoubleConsumer,
    private val valueSupplier: DoubleSupplier,
    private val kP: DoubleSupplier,
    private val kI: DoubleSupplier = DoubleSupplier { 0.0 },
    private val kD: DoubleSupplier = DoubleSupplier { 0.0 },
    private val kV: DoubleSupplier = DoubleSupplier { 0.0 },
    private val kA: DoubleSupplier = DoubleSupplier { 0.0 },
    private val kS: DoubleSupplier = DoubleSupplier { 0.0 },
    private val maxI: DoubleSupplier = DoubleSupplier { 32767.0 },
    private val errorThreshold: Double = 0.0,
    private val stopAtTarget: Boolean = false
) {
    private var currentTime = 0L
    private val startTime = System.nanoTime()
    private var error = 0.0
    var target = 0.0
        private set
    private var errorOld = 0.0
    private var uP = 0.0
    private var uI = 0.0
    private var uD = 0.0
    private var uV = 0.0
    private var uA = 0.0
    private var uS = 0.0
    private var power = 0.0
    private var timeOld = 0L
    private var timeDelta = 0.0
    private var targetOld = 0.0
    private val MAX_INT16 = 32767.0
    private var currentValue = 0.0

    /*fun updateCoefficients() {
    }*/
    fun update(target: Double): Double {
        this.target = target
        currentTime = System.nanoTime() - startTime
        timeDelta = (currentTime - timeOld) / 1000000000.0
        timeOld = currentTime
        currentValue = valueSupplier.asDouble
        error = target - currentValue
        uP = error * kP.asDouble
        uD = (error - errorOld) * kD.asDouble / timeDelta
        uI += (kI.asDouble * error) * timeDelta
        if (target == 0.0) uI = .0
        else if (abs(uI) > maxI.asDouble) uI = sign(uI) * maxI.asDouble
        uV = kV.asDouble * target
        uA = kA.asDouble * (target - targetOld) / timeDelta
        uS = kS.asDouble * sign(target)
        power = if (stopAtTarget && isAtTarget()) 0.0 else
            (uP + uI + uD + uV + uA + uS) / MAX_INT16
        errorOld = error
        targetOld = target
        outputConsumer.accept(power)
        return power
    }

    fun isAtTarget(): Boolean {
        return abs(error) < errorThreshold
    }
}