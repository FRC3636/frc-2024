package com.frcteam3636.frc2024.subsystems.shooter

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ShooterIO {
    class ShooterIOInputs: LoggableInputs {
        override fun fromLog(table: LogTable?) {

        }

        override fun toLog(table: LogTable?) {

        }
    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ShooterIOInputs?) {}

    /** Run the launcher flywheel at the specified percent speed.  */
    fun shoot(speed: Double) {}

    /** Run the launcher flywheels in reverse to intake at the specified percent speed. */
    fun intake(speed: Double) {}
}