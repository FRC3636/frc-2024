package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.can.CANSparkFlex
import com.frcteam3636.frc2024.can.CANSparkMax
import com.frcteam3636.frc2024.can.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
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
            table?.put("OTB Feed Velocity", overTheBumperFeedVelocityHz)
            table?.put("UTB Roller Velocity", underTheBumperRollersVelocityHz)
        }

        override fun fromLog(table: LogTable) {
            overTheBumperFeedVelocityHz = table.get("OTB Feed Velocity",  overTheBumperFeedVelocityHz)!![0]
            underTheBumperRollersVelocityHz = table.get("UTB Roller Velocity", underTheBumperRollersVelocityHz)!![0]
        }
    }

    fun updateInputs(inputs: IntakeInputs)

    fun setOverBumperFeed(speed: Double) {}
    fun setUnderBumperRoller(speed: Double) {}
}

class IntakeIOReal: IntakeIO {
    private var overTheBumperFeed = CANSparkMax(REVMotorControllerId.OverTheBumperIntakeFeed, CANSparkLowLevel.MotorType.kBrushless)
    private var underTheBumperRoller = CANSparkFlex(REVMotorControllerId.UnderTheBumperIntakeRoller, CANSparkLowLevel.MotorType.kBrushless)

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
