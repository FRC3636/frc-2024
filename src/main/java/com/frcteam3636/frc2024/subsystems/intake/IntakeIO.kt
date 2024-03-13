package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Current
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
    class IntakeInputs : LoggableInputs {
        var rollerVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RadiansPerSecond)
        var rollerCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)
        var beamBroken: Boolean = false

        override fun toLog(table: LogTable?) {
            table?.put("Roller Velocity", rollerVelocity)
            table?.put("Roller Current", rollerCurrent)
            table?.put("Beam Broken", beamBroken)

        }

        override fun fromLog(table: LogTable) {
            rollerVelocity = table.get("Roller Velocity", rollerVelocity)!!
            rollerCurrent = table.get("Roller Current", rollerCurrent)
            beamBroken = table.get("Beam Broken", beamBroken)
        }
    }

    fun updateInputs(inputs: IntakeInputs)

    fun setRollerDutyCycle(dutyCycle: Double)
}

class IntakeIOReal : IntakeIO {
    private var rollers =
        CANSparkFlex(
            REVMotorControllerId.UnderTheBumperIntakeRoller,
            CANSparkLowLevel.MotorType.kBrushless
        )
   private var beamBreakSensor: DigitalInput = DigitalInput(BEAM_BREAK_PORT)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.rollerVelocity.mut_setMagnitude(rollers.encoder.velocity)
        inputs.rollerCurrent.mut_setMagnitude(rollers.outputCurrent)
        inputs.beamBroken = beamBreakSensor.get()
    }

    override fun setRollerDutyCycle(dutyCycle: Double) {
        rollers.set(dutyCycle)
    }

    internal companion object Constants {
        const val BEAM_BREAK_PORT = 0
    }
}

class IntakeIOSim : IntakeIO {
    private var rollers = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, ROLLER_INERTIA)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        rollers.update(Robot.period)
        inputs.rollerVelocity.mut_setMagnitude(rollers.angularVelocityRadPerSec)
    }

    override fun setRollerDutyCycle(dutyCycle: Double) {
        val volts = (dutyCycle * 12.0).coerceIn(-12.0, 12.0)
        rollers.setInputVoltage(volts)
    }

    companion object Constants {
        const val ROLLER_INERTIA = 0.0002
    }
}
