package com.frcteam3636.frc2024.utils.math

import com.ctre.phoenix6.configs.SlotConfigs
import com.revrobotics.SparkPIDController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward

data class MotorFFGains(
    val s: Double = 0.0, val v: Double = 0.0, val a: Double = 0.0
)

fun SimpleMotorFeedforward(gains: MotorFFGains) = SimpleMotorFeedforward(gains.s, gains.v, gains.a)

val SimpleMotorFeedforward.gains: MotorFFGains
    get() = MotorFFGains(
        s = ks, v = kv, a = ka
    )

fun SlotConfigs.withMotorFFGains(gains: MotorFFGains) = withKS(gains.s).withKV(gains.v).withKA(gains.a)

data class PIDGains(
    val p: Double = 0.0, val i: Double = 0.0, val d: Double = 0.0
)

fun PIDController(gains: PIDGains) = PIDController(gains.p, gains.i, gains.d)

var PIDController.gains: PIDGains
    get() = PIDGains(
        p = p, i = i, d = d
    )
    set(gains) {
        p = gains.p
        i = gains.i
        d = gains.d
    }

var SparkPIDController.pidGains: PIDGains
    get() = PIDGains(
        p = p,
        i = i,
        d = d,
    )
    set(gains) {
        p = gains.p
        i = gains.i
        d = gains.d
    }