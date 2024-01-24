package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units


class ShooterIOReal : ShooterIO {

    companion object {
        const val FLYWHEEL_GEAR_RATIO = 1.0
    }

    private val left = CANSparkMax(REVMotorControllerId.LeftShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless).apply {
        inverted = true
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO / 60
    }

    private val right = CANSparkMax(REVMotorControllerId.RightShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless).apply {
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO / 60
        inverted = false
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        inputs.leftSpeed = Rotation2d(left.encoder.velocity)
        inputs.rightSpeed = Rotation2d(right.encoder.velocity)
    }

    override fun shoot(speed: Double) {
        left.set(speed)
        right.set(speed)
    }

    override fun intake(speed: Double) {
        left.set(-speed)
        right.set(-speed)
    }
}