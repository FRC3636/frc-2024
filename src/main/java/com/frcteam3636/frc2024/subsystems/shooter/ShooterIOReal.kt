package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANDevice
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax


class ShooterIOReal : ShooterIO {

    private val left = CANSparkMax(CANDevice.LeftShooterFlywheel.id, CANSparkLowLevel.MotorType.kBrushless)
    private val right = CANSparkMax(CANDevice.RightShooterFlywheel.id, CANSparkLowLevel.MotorType.kBrushless)

    init {
        left.inverted = true
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
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