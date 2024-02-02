package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.math.pow

interface IntakeIO {
    class IntakeInputs : LoggableInputs {
        var otbRollerVelocity = Rotation2d()
        var utbRollerVelocity = Rotation2d()
        var otbCurrent: Double = 0.0
        var utbCurrent: Double = 0.0
        var isIntaking: Boolean = false
//        var beamBreak: Boolean = false

        override fun toLog(table: LogTable?) {
            table?.put("OTB Roller Velocity", otbRollerVelocity)
            table?.put("UTB Roller Velocity", utbRollerVelocity)
            table?.put("OTB Current", otbCurrent)
            table?.put("UTB Current", utbCurrent)
            table?.put("Is Intaking", isIntaking)

        }

        override fun fromLog(table: LogTable) {
            otbRollerVelocity = table.get("OTB Roller Velocity", otbRollerVelocity)!![0]
            utbRollerVelocity = table.get("UTB Roller Velocity", utbRollerVelocity)!![0]
            otbCurrent = table.get("OTB Current", otbCurrent)
            utbCurrent = table.get("UTB Current", utbCurrent)
            isIntaking = table.get("Is Intaking", isIntaking)
        }
    }

    fun updateInputs(inputs: IntakeInputs)

    fun setOverBumperRoller(speed: Double)
    fun setUnderBumperRoller(speed: Double)
    fun isIntaking(): Boolean
}

class IntakeIOReal : IntakeIO {
    private var otbRollers =
        CANSparkMax(
            REVMotorControllerId.OverTheBumperIntakeFeed,
            CANSparkLowLevel.MotorType.kBrushless
        )
    private var utbRollers =
        CANSparkFlex(
            REVMotorControllerId.UnderTheBumperIntakeRoller,
            CANSparkLowLevel.MotorType.kBrushless
        )
//    private var beamBreakSensor: DigitalInput = DigitalInput(Constants.BEAM_BREAK_PORT)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.otbRollerVelocity = Rotation2d(otbRollers.encoder.velocity)
        inputs.utbRollerVelocity = Rotation2d(utbRollers.encoder.velocity)
        inputs.otbCurrent = otbRollers.outputCurrent
        inputs.utbCurrent = utbRollers.outputCurrent
        inputs.isIntaking = isIntaking()
        //      inputs.beamBreak = beamBreakSensor.get()
    }

    override fun setOverBumperRoller(speed: Double) {
        otbRollers.set(speed)
    }

    override fun setUnderBumperRoller(speed: Double) {
        utbRollers.set(speed)
    }

    override fun isIntaking(): Boolean {
//        return beamBreakSensor.get() &&
        return utbRollers.outputCurrent > Constants.BEAM_BREAK_CURRENT_THRESHOLD
    }

    internal companion object Constants {
        const val BEAM_BREAK_PORT = 0
        const val BEAM_BREAK_CURRENT_THRESHOLD = 10.0
    }





}

class IntakeIOSim : IntakeIO {
    companion object {
        const val ROLLER_INERTIA = 0.0002
    }

    private var otbRollers = FlywheelSim(DCMotor.getNEO(1), 1.0, ROLLER_INERTIA)
    private var utbRollers = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, ROLLER_INERTIA)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        otbRollers.update(Robot.period)
        utbRollers.update(Robot.period)
        inputs.otbRollerVelocity = Rotation2d(otbRollers.angularVelocityRadPerSec)
        inputs.utbRollerVelocity = Rotation2d(utbRollers.angularVelocityRadPerSec)
    }

    override fun setOverBumperRoller(speed: Double) {
        val volts = (speed * 12.0).coerceIn(-12.0, 12.0)
        otbRollers.setInputVoltage(volts)
    }

    override fun setUnderBumperRoller(speed: Double) {
        val volts = (speed * 12.0).coerceIn(-12.0, 12.0)
        utbRollers.setInputVoltage(volts)
    }

    override fun isIntaking(): Boolean {
        return false
    }
}
