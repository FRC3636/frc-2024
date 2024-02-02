package com.frcteam3636.frc2024.subsystems.climber

import com.frcteam3636.frc2024.Robot
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

class Climber : Subsystem {
    private var io: ClimberIO = when (Robot.model) {
        Robot.Model.PRACTICE -> ClimberIOReal()
        Robot.Model.COMPETITION -> ClimberIOReal()
        Robot.Model.SIMULATION -> TODO()
    }
    var inputs = ClimberIO.ClimberInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climber", inputs)
    }
}
