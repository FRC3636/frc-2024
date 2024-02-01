package com.frcteam3636.frc2024.utils.math

import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.PI

const val TAU = PI * 2

fun Translation2d.fromPolar(magnitude: Double, angle: Double): Translation2d {
    return Translation2d(magnitude * Math.cos(angle), magnitude * Math.sin(angle))
}
