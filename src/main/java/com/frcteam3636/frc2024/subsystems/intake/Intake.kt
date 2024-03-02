package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.Robot
import edu.wpi.first.wpilibj2.command.*
import org.littletonrobotics.junction.Logger

object Intake : Subsystem {
    private var io: IntakeIO = when (Robot.model) {
        Robot.Model.SIMULATION -> IntakeIOSim()
        Robot.Model.COMPETITION -> IntakeIOReal()
        Robot.Model.PRACTICE -> IntakeIOReal()
    }

    var inputs = IntakeIO.IntakeInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }

    fun intakeCommand(): Command {
        return startEnd(
            {
                io.setUnderBumperRoller(0.5)
                io.setOverBumperRoller(0.5)
            },
            {
                io.setUnderBumperRoller(0.0)
                io.setOverBumperRoller(0.0)
            }
        )
            .until(inputs::isIntaking)

    }

    fun indexCommand(): Command {
        return Commands.sequence(
            InstantCommand ( {io.setUnderBumperRoller(0.8)}),
            WaitCommand(4.0),
        ).finallyDo(Runnable {
            io.setUnderBumperRoller(0.0)
        })
    }
}
