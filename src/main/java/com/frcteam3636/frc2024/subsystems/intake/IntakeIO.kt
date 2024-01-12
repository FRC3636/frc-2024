package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.CANDevice
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        var armAngle = Rotation2d()
        var armAngularVelocityHz = Rotation2d()
    }

    fun updateInputs(inputs: IntakeInputs)

    fun setOverBumperFeed(speed: Double) {}
    fun setUnderBumperRoller(speed: Double) {}
    fun moveArm(speed: Double) {}
}

class IntakeIOReal: IntakeIO {
    companion object {
        const val ARM_GEAR_RATIO = 1.0
    }

    private var armMotor = CANSparkMax(CANDevice.OverTheBumperIntakeArm.id, CANSparkLowLevel.MotorType.kBrushless)
    private val armEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

    private var overTheBumperFeed = CANSparkMax(CANDevice.OverTheBumperIntakeFeed.id, CANSparkLowLevel.MotorType.kBrushless)
    private var underTheBumperRoller = CANSparkFlex(CANDevice.UnderTheBumperIntakeRoller.id, CANSparkLowLevel.MotorType.kBrushless)

    init {
        armEncoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * ARM_GEAR_RATIO / 60
        armEncoder.positionConversionFactor = Units.rotationsToRadians(1.0) * ARM_GEAR_RATIO

        armMotor.burnFlash()
    }
    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.armAngle = Rotation2d(armEncoder.position)
        inputs.armAngularVelocityHz = Rotation2d(armEncoder.velocity)
    }

    override fun setOverBumperFeed(speed: Double) {
        overTheBumperFeed.set(speed)
    }

    override fun setUnderBumperRoller(speed: Double) {
        underTheBumperRoller.set(speed)
    }

    override fun moveArm(speed: Double) {
        armMotor.set(speed)
    }
}