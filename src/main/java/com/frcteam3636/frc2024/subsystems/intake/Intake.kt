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

    fun outtake(): Command =
        startEnd(
            {
                io.setRollerDutyCycle(-0.2)
            },
            {
                io.setRollerDutyCycle(0.0)
            }
        )

    fun intake(): Command =
        startEnd(
            {
                io.setRollerDutyCycle(0.7)
            },
            {
                io.setRollerDutyCycle(0.0)
            }
        ).until(inputs::beamBroken)

    fun index(): Command =
        Commands.sequence(
            InstantCommand({ io.setRollerDutyCycle(0.5) }),
            WaitCommand(3.0),
        ).finallyDo(Runnable {
            io.setRollerDutyCycle(0.0)
        })
}
