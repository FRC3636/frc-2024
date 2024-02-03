package com.frcteam3636.frc2024.subsystems.climber

import com.frcteam3636.frc2024.Robot
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

class Climber : Subsystem {
    private var io: ClimberIO = when (Robot.model) {
        Robot.Model.PRACTICE -> ClimberIOReal()
        Robot.Model.COMPETITION -> ClimberIOReal()
        Robot.Model.SIMULATION -> ClimberIOSim()
    }
    var inputs = ClimberIO.ClimberInputs()

    private var mech = Mechanism2d(3.0, 3.0);
    private var climberRoot = mech.getRoot("climber", 1.5, 0.0)
    private var elevatorLigament = climberRoot.append(MechanismLigament2d("elevator", 0.0, 90.0))

    init {
        SmartDashboard.putData("Climber Mechanism", mech)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        elevatorLigament.length = inputs.climberPosition
        Logger.processInputs("Climber", inputs)
    }
}
