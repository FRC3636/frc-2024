package com.frcteam3636.frc2024.subsystems.shooter

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Shooter : Subsystem {
    private val io: ShooterIO = if (RobotBase.isReal()) {
        ShooterIOReal()
    } else {
        TODO()
    }

    val inputs = ShooterIO.ShooterIOInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)
    }

    fun shootCommand(): Command {
        return startEnd({
            io.shoot(1.0)
        }, {
            io.shoot(0.0)
        })
    }

    fun intakeCommand(): Command {
        return startEnd({
            io.intake(1.0)
        }, {
            io.intake(0.0)
        })
    }
}