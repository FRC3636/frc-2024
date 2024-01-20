package com.frcteam3636.frc2024.subsystems.climber

import com.frcteam3636.frc2024.can.CANSparkMax
import com.frcteam3636.frc2024.can.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.SparkAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.AutoLog

interface ClimberIO {
    @AutoLog
    class ClimberInputs {
        var climberPosition = Rotation2d()
    }

    fun updateInputs(inputs: ClimberInputs)

    fun moveClimber(speed: Double) {}
}
class ClimberIOReal: ClimberIO {
    companion object {
        const val CLIMBER_GEAR_RATIO = 1.0
    }


    private var climberMotor = CANSparkMax(REVMotorControllerId.ClimberMotor, CANSparkLowLevel.MotorType.kBrushless).apply{
        burnFlash()
    }
    private val climberEncoder = climberMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).apply {
        velocityConversionFactor = Units.rotationsToRadians(1.0) * CLIMBER_GEAR_RATIO / 60
        positionConversionFactor = Units.rotationsToRadians(1.0) * CLIMBER_GEAR_RATIO
    }

    override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
        inputs.climberPosition = Rotation2d(climberEncoder.position)
    }


    override fun moveClimber(speed: Double) {
        climberMotor.set(speed)
    }
}