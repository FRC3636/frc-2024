package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.Robot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
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

    fun outtake(): Command =
        startEnd(
            {
                io.setUnderBumperRoller(-0.5)
            },
            {
                io.setUnderBumperRoller(0.0)
            }
        )

    fun intake(): Command =
        startEnd(
            {
                io.setUnderBumperRoller(0.7)
            },
            {
                io.setUnderBumperRoller(0.0)
            }
        ).until(inputs::isIntaking)

    fun index(): Command = startEnd({
            io.setUnderBumperRoller(0.5)
        }, {
            io.setUnderBumperRoller(0.0)
        })
        .withTimeout(3.0)
}
