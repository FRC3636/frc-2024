package com.frcteam3636.frc2024.subsystems.climber


import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Subsystem

class Climber: Subsystem {
    private var io: ClimberIO = if (RobotBase.isReal()) {
        ClimberIOReal()
    } else {
        TODO()
    }
    var inputs = ClimberIO.ClimberInputs()

    override fun periodic() {
        io.updateInputs(inputs)
    }
}