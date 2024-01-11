package com.frcteam3636.frc2024.subsystems.shooter

import edu.wpi.first.wpilibj.RobotBase

object Shooter {
    private val io: ShooterIO = if (RobotBase.isReal()) {
        ShooterIOReal()
    }  else {
        TODO()
    }
}