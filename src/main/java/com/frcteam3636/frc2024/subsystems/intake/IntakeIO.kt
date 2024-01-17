package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.CANDevice
import com.frcteam3636.frc2024.Robot
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
    class IntakeInputs: LoggableInputs {
        var overTheBumperFeedVelocityHz = Rotation2d()
        var underTheBumperRollersVelocityHz = Rotation2d()

        override fun toLog(table: LogTable?) {
            table?.put("OTB Feed Velocity", overTheBumperFeedVelocityHz.radians)
            table?.put("UTB Roller Velocity", underTheBumperRollersVelocityHz.radians)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("OTB Feed Velocity", 0.0)?.let { overTheBumperFeedVelocityHz = Rotation2d.fromRadians(it) }
            table?.get("UTB Roller Velocity", 0.0)?.let { underTheBumperRollersVelocityHz = Rotation2d.fromRadians(it) }
        }
    }

    fun updateInputs(inputs: IntakeInputs)

    fun setOverBumperFeed(speed: Double) {}
    fun setUnderBumperRoller(speed: Double) {}
}

class IntakeIOReal: IntakeIO {
    private var overTheBumperFeed = CANSparkMax(CANDevice.OverTheBumperIntakeFeed.id, CANSparkLowLevel.MotorType.kBrushless)
    private var underTheBumperRoller = CANSparkFlex(CANDevice.UnderTheBumperIntakeRoller.id, CANSparkLowLevel.MotorType.kBrushless)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.overTheBumperFeedVelocityHz = Rotation2d(overTheBumperFeed.encoder.velocity)
        inputs.underTheBumperRollersVelocityHz = Rotation2d(underTheBumperRoller.encoder.velocity)
    }

    override fun setOverBumperFeed(speed: Double) {
        overTheBumperFeed.set(speed)
    }

    override fun setUnderBumperRoller(speed: Double) {
        underTheBumperRoller.set(speed)
    }
}

class IntakeIOSim: IntakeIO {
    companion object {
        const val ROLLER_INERTIA = 0.0002
    }

    private var overTheBumperFeed = FlywheelSim(DCMotor.getNEO(1), 1.0, ROLLER_INERTIA)
    private var underTheBumperRoller = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, ROLLER_INERTIA)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        overTheBumperFeed.update(Robot.period)
        underTheBumperRoller.update(Robot.period)
        inputs.overTheBumperFeedVelocityHz = Rotation2d(overTheBumperFeed.angularVelocityRadPerSec)
        inputs.underTheBumperRollersVelocityHz = Rotation2d(underTheBumperRoller.angularVelocityRadPerSec)
    }

    override fun setOverBumperFeed(speed: Double) {
        val volts = (speed * 12.0).coerceIn(-12.0, 12.0)
        overTheBumperFeed.setInputVoltage(volts)
    }

    override fun setUnderBumperRoller(speed: Double) {
        val volts = (speed * 12.0).coerceIn(-12.0, 12.0)
        underTheBumperRoller.setInputVoltage(volts)
    }
}
