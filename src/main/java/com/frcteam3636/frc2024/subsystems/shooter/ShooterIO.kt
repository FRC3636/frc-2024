package com.frcteam3636.frc2024.subsystems.shooter

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.math.geometry.Rotation2d


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
    fun updateInputs(inputs: ShooterIOInputs?) {}

    /** Run the launcher flywheel at the specified percent speed.  */
    fun shoot(speed: Double) {}

    /** Run the launcher flywheels in reverse to intake at the specified percent speed. */
    fun intake(speed: Double) {}
}