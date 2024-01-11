package com.frcteam3636.frc2024.subsystems.shooter

import org.littletonrobotics.junction.AutoLog


interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {

    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ShooterIOInputs?) {}

    /** Run the launcher wheel at the specified voltage.  */
    fun setLaunchVoltage(volts: Double) {}

    /** Run the feeder wheel at the specified voltage.  */
    fun setFeedVoltage(volts: Double) {}
}