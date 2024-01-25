package com.frcteam3636.frc2024.subsystems.intake

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
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
        return InstantCommand(
            {
                io.setUnderBumperRoller(1.0)
                io.setOverBumperRoller(1.0)
            }
        ).until(io::isIntaking).also { it.addRequirements(this) }
    }

    fun indexCommand(): Command {
        return SequentialCommandGroup(
            runOnce({ io.setUnderBumperRoller(1.0) }),
            WaitCommand(1.0),
            runOnce({ io.setUnderBumperRoller(0.0) })
        )
    }

    
}
