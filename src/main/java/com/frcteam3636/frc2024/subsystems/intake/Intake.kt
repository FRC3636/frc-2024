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
        return StartEndCommand(
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
            .also { it.addRequirements(this) }
    }



    fun indexCommand(): Command {
        return SequentialCommandGroup(
            runOnce { io.setUnderBumperRoller(0.4) },
            WaitCommand(1.0),
            runOnce { io.setUnderBumperRoller(0.0) }
        )
    }
}
