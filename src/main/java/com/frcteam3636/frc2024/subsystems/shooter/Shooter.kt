package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.utils.math.*
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PIDCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Shooter : Subsystem {
    private val io: ShooterIO =
            if (RobotBase.isReal()) {
                ShooterIOReal()
            } else {
                TODO()
            }

    private val pidController = PIDController(PIDGains(0.1, 0.0, 0.0))

    val inputs = ShooterIO.ShooterIOInputs()

    val tab = Shuffleboard.getTab("Shooter")
    val shouldSpin = tab.add("Should Spin", true).withWidget(BuiltInWidgets.kToggleSwitch).entry

    val targetVelocity =
            tab.add("Target Velocity", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(
                            mapOf(Pair("min", 0.0), Pair("max", 5000.0))
                    ) // Adjust min and max as needed.
                    .entry

    private val rateLimiter = SlewRateLimiter(1.0)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)
    }

    fun shootCommand(): Command {
        return PIDCommand(
                        pidController,
                        { inputs.leftSpeed.radians },
                        { targetVelocity.getDouble(0.0) },
                        { output ->
                            val limitedOutput = rateLimiter.calculate(output)
                            io.shoot(limitedOutput, shouldSpin.getBoolean(true))
                        }
                )
                .also { it.addRequirements(this) }
    }

    fun intakeCommand(): Command {
        return startEnd({ io.intake(1.0) }, { io.intake(0.0) })
    }
}
