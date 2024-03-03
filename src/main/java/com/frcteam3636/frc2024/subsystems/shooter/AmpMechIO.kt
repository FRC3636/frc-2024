package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.utils.math.TAU
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
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

    fun setVoltage(volts: Measure<Voltage>) {}

    fun zero() {}

}

class AmpMechIOReal: AmpMechIO {

    val pivotMotor = CANSparkFlex(REVMotorControllerId.AmpMech, CANSparkLowLevel.MotorType.kBrushless).apply{
        restoreFactoryDefaults()
        inverted = false;
        encoder.positionConversionFactor = TAU * GEAR_RATIO
        encoder.velocityConversionFactor = ( TAU * GEAR_RATIO  )/ 60
    }

    private val pid = pivotMotor.pidController.apply {
        p = 0.1
        i = 0.0
        d = 0.0
    }

    override fun updateInputs(inputs: AmpMechIO.Inputs) {

        inputs.current =  pivotMotor.outputCurrent
        if(inputs.current > CURRENT_LIMIT){
            pivotMotor.encoder.position = 0.0
        }
        inputs.position = Rotation2d(pivotMotor.encoder.position)

    }

    override fun pivotTo(position: Rotation2d) {
        Logger.recordOutput("Shooter/Amp Mech Setpoint", position)
        pid.setReference(
            position.radians, CANSparkBase.ControlType.kPosition
        )
    }

    override fun zero(){
        pivotMotor.encoder.position = 0.0
    }

    override fun setVoltage(volts: Measure<Voltage>) {
        pivotMotor.setVoltage(volts.baseUnitMagnitude())
    }



    internal companion object Constants {
        const val GEAR_RATIO: Double = 2.0 / 3
        const val CURRENT_LIMIT: Double = 120.0

    }

}