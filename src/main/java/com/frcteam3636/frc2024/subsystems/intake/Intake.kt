package com.frcteam3636.frc2024.subsystems.intake

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Intake: Subsystem {
    private var io: IntakeIO = if (RobotBase.isReal()) {
        IntakeIOReal()
    } else {
        IntakeIOSim()
    }

    var inputs = IntakeIO.IntakeInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }

    fun intakeCommand(): Command {
        return startEnd({
            io.setOverBumperFeed(1.0)
            io.setUnderBumperRoller(1.0)
        }, {
            io.setOverBumperFeed(0.0)
            io.setUnderBumperRoller(0.0)
        })
    }
}
