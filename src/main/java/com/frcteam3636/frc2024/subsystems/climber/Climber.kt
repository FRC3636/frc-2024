package com.frcteam3636.frc2024.subsystems.climber

import com.frcteam3636.frc2024.BLACK
import com.frcteam3636.frc2024.BLUE
import com.frcteam3636.frc2024.Robot
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.Command
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

    fun extendClimberCommand(): Command {
        return runOnce {
            io.moveClimber(1.0)
        }
            .until { inputs.climberPosition >= extendedPosition }
            .finallyDo(Runnable {
                io.moveClimber(0.0)
            })
    }

    fun retractClimberCommand(): Command {
        return runOnce {
            io.moveClimber(-1.0)
        }
            .until { inputs.climberPosition <= retractedPosition }
            .finallyDo(Runnable {
                io.moveClimber(0.0)
            })
    }


}
