package com.frcteam3636.frc2024.subsystems.shooter

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Shooter : Subsystem {
    private val io: ShooterIO = if (RobotBase.isReal()) {
        ShooterIOReal()
    } else {
        TODO()
    }

    val inputs = ShooterIO.ShooterIOInputs()

    val tab = Shuffleboard.getTab("Shooter")
    val shouldSpin = tab.add("Should Spin", true).withWidget(BuiltInWidgets.kBooleanBox).entry

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)
    }

    fun shootCommand(): Command {
        val accelTimer = Timer()
        return FunctionalCommand({
            accelTimer.start()
        }, {
            io.shoot(accelTimer.get().coerceIn(0.0, 1.0), shouldSpin.getBoolean(true))
        }, {
            io.shoot(0.0, shouldSpin.getBoolean(true))
            accelTimer.stop()
            accelTimer.reset()
        }, { false}, this)
    }

    fun intakeCommand(): Command {
        return startEnd({
            io.intake(1.0)
        }, {
            io.intake(0.0)
        })
    }
}