package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.utils.math.TAU
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface AmpMechIO {

    public class Inputs: LoggableInputs {

        var position: Rotation2d = Rotation2d()
        var current: Double = 0.0

        override fun fromLog(table: LogTable?){
            position = table?.get("Position", Rotation2d())!![0]
            current = table.get("Current", 0.0)
        }

        override fun toLog(table: LogTable?){
            table?.put("Current", current)
            table?.put("Position", position)
        }

    }

    fun updateInputs(inputs: Inputs)

    fun pivotTo(position: Rotation2d)

}

class AmpMechIOReal: AmpMechIO {

    val pivotMotor = CANSparkFlex(REVMotorControllerId.AmpMech, CANSparkLowLevel.MotorType.kBrushless).apply{
        restoreFactoryDefaults()
        inverted = false;
        encoder.positionConversionFactor = TAU * GEAR_RATIO
        encoder.velocityConversionFactor = ( TAU * GEAR_RATIO  ) / 60
    }

    private val pid = pivotMotor.pidController.apply {
        p = 0.1
        i = 0.0
        d = 0.0
    }

    override fun updateInputs(inputs: AmpMechIO.Inputs) {
        pivotMotor.encoder.position
        pivotMotor.outputCurrent
    }

    override fun pivotTo(position: Rotation2d) {
        pid.setReference(
            position.radians, CANSparkBase.ControlType.kPosition
        )
    }

    internal companion object Constants {
        const val GEAR_RATIO: Double = 2.0 / 3
    }

}