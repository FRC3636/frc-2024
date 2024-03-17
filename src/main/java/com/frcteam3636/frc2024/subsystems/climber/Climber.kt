package com.frcteam3636.frc2024.subsystems.climber

import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2024.BLACK
import com.frcteam3636.frc2024.BLUE
import com.frcteam3636.frc2024.Robot
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Climber : Subsystem {
    private var io: ClimberIO = when (Robot.model) {
        Robot.Model.PRACTICE -> ClimberIOReal()
        Robot.Model.COMPETITION -> ClimberIOReal()
        Robot.Model.SIMULATION -> ClimberIOSim()
    }
    var inputs = ClimberIO.ClimberInputs()

    private var mech = Mechanism2d(2.0, 2.0, BLACK)
    private var climberRoot = mech.getRoot("climber", 1.0, 0.0)
    private var elevatorLigament = climberRoot.append(MechanismLigament2d("elevator", 0.0, 90.0, 10.0, BLUE))

    const val extendedPosition = 1.0 //todo: find this
    const val retractedPosition = 0.0 //todo: find this

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climber", inputs)
        elevatorLigament.length = inputs.climberPosition
        Logger.recordOutput("Climber", mech)
    }

    fun setClimber(speed: Double): Command =
        runEnd({
            io.setNeutral(NeutralModeValue.Coast)
            io.moveClimber(speed)
        }, {
            io.moveClimber(0.0)
            io.setNeutral(NeutralModeValue.Brake)
        }
        )

    fun knockIntake(): Command =
        Commands.sequence(
            setClimber(0.9).withTimeout(0.3),
            setClimber(-0.8).withTimeout(0.5),
            setClimber(0.0).withTimeout(0.1)
        )

    fun extendClimber(): Command =
         FunctionalCommand(
            {},
            { io.moveClimber(1.0) },
            { io.moveClimber(0.0) },
            // wait until climber is extended
            { inputs.climberPosition <= extendedPosition },
            this
        )


    fun retractClimber(): Command =
        FunctionalCommand(
            {},
            { io.moveClimber(-1.0) },
            { io.moveClimber(0.0) },
            // wait until climber is retracted
            { inputs.climberPosition <= retractedPosition },
            this
        )

}
