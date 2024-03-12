package com.frcteam3636.frc2024.subsystems.climber

import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2024.BLACK
import com.frcteam3636.frc2024.BLUE
import com.frcteam3636.frc2024.Robot
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.*
import org.littletonrobotics.junction.Logger

object Climber : Subsystem {
    private var io: ClimberIO = when (Robot.model) {
        Robot.Model.PRACTICE -> ClimberIOReal()
        Robot.Model.COMPETITION -> ClimberIOReal()
        Robot.Model.SIMULATION -> ClimberIOSim()
    }
    var inputs = ClimberIO.Inputs()

    private var mech = Mechanism2d(2.0, 2.0, BLACK)
    private var climberRoot = mech.getRoot("climber", 1.0, 0.0)
    private var elevatorLigament = climberRoot.append(MechanismLigament2d("elevator", 0.0, 90.0, 10.0, BLUE))

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climber", inputs)
        elevatorLigament.length = inputs.climberPosition.baseUnitMagnitude()
        Logger.recordOutput("Climber", mech)
    }

    fun runClimber(speed: Double): Command =
        runEnd({
            io.setNeutral(NeutralModeValue.Coast)
            io.setDutyCycle(speed)
        }, {
            io.setDutyCycle(0.0)
            io.setNeutral(NeutralModeValue.Brake)
        })

    fun knockIntake(): Command =
        Commands.sequence(
            runClimber(0.9).withTimeout(0.3),
            runClimber(-0.8).withTimeout(0.5),
            runClimber(0.0).withTimeout(0.1)
        )
}
