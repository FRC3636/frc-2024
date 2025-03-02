package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
    class IntakeInputs : LoggableInputs {
        var utbRollerVelocity = Rotation2d()
        var utbCurrent: Double = 0.0
        var isIntaking: Boolean = false
        var beamBreak: Boolean = false

        override fun toLog(table: LogTable?) {
            table?.put("UTB Roller Velocity", utbRollerVelocity)
            table?.put("UTB Current", utbCurrent)
            table?.put("Is Intaking", isIntaking)
            table?.put("Beam Break", beamBreak)

        }

        override fun fromLog(table: LogTable) {
            utbRollerVelocity = table.get("UTB Roller Velocity", utbRollerVelocity)!![0]
            utbCurrent = table.get("UTB Current", utbCurrent)
            isIntaking = table.get("Is Intaking", isIntaking)
            beamBreak = table.get("Beam Break", beamBreak)
        }
    }

    fun updateInputs(inputs: IntakeInputs)

    fun setUnderBumperRoller(speed: Double)
}

class IntakeIOReal : IntakeIO {
    private var utbRollers =
        CANSparkFlex(
            REVMotorControllerId.UnderTheBumperIntakeRoller,
            CANSparkLowLevel.MotorType.kBrushless
        )
    private var beamBreakSensor: DigitalInput = DigitalInput(BEAM_BREAK_PORT)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.utbRollerVelocity = Rotation2d(utbRollers.encoder.velocity)
        inputs.utbCurrent = utbRollers.outputCurrent
        inputs.beamBreak = beamBreakSensor.get()
        inputs.isIntaking = !inputs.beamBreak
    }

    override fun setUnderBumperRoller(speed: Double) {
        utbRollers.set(speed)
    }

    internal companion object Constants {
        const val BEAM_BREAK_PORT = 0
    }
}

class IntakeIOSim : IntakeIO {
    private var utbRollers = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, ROLLER_INERTIA)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        utbRollers.update(Robot.period)
        inputs.utbRollerVelocity = Rotation2d(utbRollers.angularVelocityRadPerSec)
        inputs.isIntaking = true
    }

    override fun setUnderBumperRoller(speed: Double) {
        val volts = (speed * 12.0).coerceIn(-12.0, 12.0)
        utbRollers.setInputVoltage(volts)
    }

    companion object Constants {
        const val ROLLER_INERTIA = 0.0002
    }
}
