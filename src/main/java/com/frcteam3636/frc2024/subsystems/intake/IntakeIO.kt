package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.CANSparkMax
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
        var otbRollerVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RadiansPerSecond)
        var utbRollerVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RadiansPerSecond)
        var otbCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)
        var utbCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)
        var beamBroken: Boolean = false

        override fun toLog(table: LogTable?) {
            table?.put("OTB Roller Velocity", otbRollerVelocity)
            table?.put("UTB Roller Velocity", utbRollerVelocity)
            table?.put("OTB Current", otbCurrent)
            table?.put("UTB Current", utbCurrent)
            table?.put("Beam Break", beamBroken)

        }

        override fun fromLog(table: LogTable) {
            otbRollerVelocity = table.get("OTB Roller Velocity", otbRollerVelocity)!!
            utbRollerVelocity = table.get("UTB Roller Velocity", utbRollerVelocity)!!
            otbCurrent = table.get("OTB Current", otbCurrent)
            utbCurrent = table.get("UTB Current", utbCurrent)
            beamBroken = table.get("Beam Break", beamBroken)
        }
    }

    fun updateInputs(inputs: IntakeInputs)

    fun setOverBumperRoller(speed: Double)
    fun setUnderBumperRoller(speed: Double)
}

class IntakeIOReal : IntakeIO {
    private var otbRollers =
        CANSparkMax(
            REVMotorControllerId.OverTheBumperIntakeFeed,
            CANSparkLowLevel.MotorType.kBrushless
        ).apply{
            inverted = true
        }
    private var utbRollers =
        CANSparkFlex(
            REVMotorControllerId.UnderTheBumperIntakeRoller,
            CANSparkLowLevel.MotorType.kBrushless
        )
   private var beamBreakSensor: DigitalInput = DigitalInput(BEAM_BREAK_PORT)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.otbRollerVelocity.mut_setMagnitude(otbRollers.encoder.velocity)
        inputs.utbRollerVelocity.mut_setMagnitude(utbRollers.encoder.velocity)
        inputs.otbCurrent.mut_setMagnitude(otbRollers.outputCurrent)
        inputs.utbCurrent.mut_setMagnitude(utbRollers.outputCurrent)
        inputs.beamBroken = beamBreakSensor.get()
    }

    override fun setOverBumperRoller(speed: Double) {
        otbRollers.set(speed)
    }

    override fun setUnderBumperRoller(speed: Double) {
        utbRollers.set(speed)
    }

    internal companion object Constants {
        const val BEAM_BREAK_PORT = 0
    }
}

class IntakeIOSim : IntakeIO {
    private var otbRollers = FlywheelSim(DCMotor.getNEO(1), 1.0, ROLLER_INERTIA)
    private var utbRollers = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, ROLLER_INERTIA)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        otbRollers.update(Robot.period)
        utbRollers.update(Robot.period)
        inputs.otbRollerVelocity.mut_setMagnitude(otbRollers.angularVelocityRadPerSec)
        inputs.utbRollerVelocity.mut_setMagnitude(utbRollers.angularVelocityRadPerSec)
    }

    override fun setOverBumperRoller(speed: Double) {
        val volts = (speed * 12.0).coerceIn(-12.0, 12.0)
        otbRollers.setInputVoltage(volts)
    }

    override fun setUnderBumperRoller(speed: Double) {
        val volts = (speed * 12.0).coerceIn(-12.0, 12.0)
        utbRollers.setInputVoltage(volts)
    }

    companion object Constants {
        const val ROLLER_INERTIA = 0.0002
    }
}
