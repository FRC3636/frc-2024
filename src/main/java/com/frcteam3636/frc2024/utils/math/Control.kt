package com.frcteam3636.frc2024.utils.math

import com.ctre.phoenix6.configs.*
import com.revrobotics.SparkPIDController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward

data class MotorFFGains(val s: Double = 0.0, val v: Double = 0.0, val a: Double = 0.0)

fun SimpleMotorFeedforward(gains: MotorFFGains) = SimpleMotorFeedforward(gains.s, gains.v, gains.a)

val SimpleMotorFeedforward.gains: MotorFFGains
    get() = MotorFFGains(s = ks, v = kv, a = ka)

var SlotConfigs.motorFFGains: MotorFFGains
    get() = MotorFFGains(s = kS, v = kV, a = kA)
    set(gains) {
        kS = gains.s
        kV = gains.v
        kA = gains.a
    }
var Slot0Configs.motorFFGains: MotorFFGains
    get() = MotorFFGains(s = kS, v = kV, a = kA)
    set(gains) {
        kS = gains.s
        kV = gains.v
        kA = gains.a
    }

data class PIDGains(val p: Double = 0.0, val i: Double = 0.0, val d: Double = 0.0)

fun PIDController(gains: PIDGains) = PIDController(gains.p, gains.i, gains.d)

var PIDController.gains: PIDGains
    get() = PIDGains(p = p, i = i, d = d)
    set(gains) {
        p = gains.p
        i = gains.i
        d = gains.d
    }

var SparkPIDController.pidGains: PIDGains
    get() =
        PIDGains(
            p = p,
            i = i,
            d = d,
        )
    set(gains) {
        p = gains.p
        i = gains.i
        d = gains.d
    }

var SlotConfigs.pidGains: PIDGains
    get() = PIDGains(p = kP, i = kI, d = kD)
    set(gains) {
        kP = gains.p
        kI = gains.i
        kD = gains.d
    }
var Slot0Configs.pidGains: PIDGains
    get() = PIDGains(p = kP, i = kI, d = kD)
    set(gains) {
        kP = gains.p
        kI = gains.i
        kD = gains.d
    }