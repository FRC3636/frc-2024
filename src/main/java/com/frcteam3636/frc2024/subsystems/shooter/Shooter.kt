package com.frcteam3636.frc2024.subsystems.shooter

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

object Shooter: Subsystem {
    private val io: ShooterIO = if (RobotBase.isReal()) {
        ShooterIOReal()
    }  else {
        ShooterIOSim()
    }

    val inputs = ShooterIO.ShooterIOInputs()

    val pivot = let {
        val mechanism = Mechanism2d(4.0, 4.0)
        val root = mechanism.getRoot("flywheels", 0.0, 0.0)
        val pivot = root.append(MechanismLigament2d("pivot", 2.0, 0.0))

        SmartDashboard.putData("Shooter", mechanism)

        pivot
    }

    override fun periodic() {
        io.updateInputs(inputs)
        //TODO set pivot angle
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