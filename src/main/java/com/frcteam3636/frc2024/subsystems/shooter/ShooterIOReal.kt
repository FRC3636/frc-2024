package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.can.*
import com.revrobotics.CANSparkLowLevel


class ShooterIOReal : ShooterIO {

    private val left = CANSparkMax(REVMotorControllerId.LeftShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless)
    private val right = CANSparkMax(REVMotorControllerId.RightShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless)

    init {
        left.inverted = true
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs?) {
        // no-op
    }

    override fun shoot(speed: Double) {
        left.set(speed)
        right.set(speed * 0.5)
    }

    override fun intake(speed: Double) {
        left.set(-speed)
        right.set(-speed)
    }
}