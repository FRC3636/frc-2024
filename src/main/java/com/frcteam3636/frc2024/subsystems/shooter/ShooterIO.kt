package com.frcteam3636.frc2024.subsystems.shooter

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.AutoLog
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ShooterIO {
    class ShooterIOInputs: LoggableInputs {
        var leftSpeed = Rotation2d()
        var rightSpeed = Rotation2d()

        override fun toLog(table: LogTable) {
            table.put("Left Speed", leftSpeed)
            table.put("Right Speed", rightSpeed)
        }

        override fun fromLog(table: LogTable) {
            leftSpeed = table.get("Left Speed", leftSpeed)[0]
            rightSpeed = table.get("Right Speed", rightSpeed)[0]
        }
    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ShooterIOInputs) {}

    /** Run the launcher flywheel at the specified percent speed.  */
    fun shoot(speed: Double) {}

    /** Run the launcher flywheels in reverse to intake at the specified percent speed. */
    fun intake(speed: Double) {}
}

class ShooterIOSim : ShooterIO {
    companion object {
        private const val LEFT_MOMENT_OF_INERTIA = 0.00005
        private const val RIGHT_MOMENT_OF_INERTIA = 0.00005
    }

    private val left = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, LEFT_MOMENT_OF_INERTIA)
    private val right = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, RIGHT_MOMENT_OF_INERTIA)

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        left.update(0.02)
        right.update(0.02)
    }

    override fun shoot(speed: Double) {
        left.setInputVoltage(speed * 12)
        right.setInputVoltage(speed * 12 * 0.5)
    }

    override fun intake(speed: Double) {
        left.setInputVoltage(-speed * 12)
        right.setInputVoltage(-speed * 12)
    }
}