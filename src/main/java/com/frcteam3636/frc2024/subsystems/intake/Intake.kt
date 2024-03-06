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
                println("Intaking")
                io.setUnderBumperRoller(1.0)
                io.setOverBumperRoller(0.5)
            },
            {
                println("Intaking done")
                io.setUnderBumperRoller(0.0)
                io.setOverBumperRoller(0.0)
            }
        )


    }

    fun indexCommand(): Command {
        return Commands.sequence(
            PrintCommand("Indexing"),
            InstantCommand ( {io.setUnderBumperRoller(0.5)}),
            WaitCommand(3.0),
            PrintCommand("Indexing done"),
        ).finallyDo(Runnable {
            io.setUnderBumperRoller(0.0)
        })
    }
}
