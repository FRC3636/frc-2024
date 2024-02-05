package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.BLACK
import com.frcteam3636.frc2024.BLUE
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.WHITE
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

object Shooter {
    object Flywheels : Subsystem {
        private val io: FlywheelIO = when (Robot.model) {
            Robot.Model.SIMULATION -> FlywheelIOSim()
            Robot.Model.COMPETITION, Robot.Model.PRACTICE -> FlywheelIOReal()
        }
        private val inputs = FlywheelIO.Inputs()

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Flywheels", inputs)

            flywheelLigament.color = if (inputs.leftSpeed.rotations > 1.0) {
                BLUE
            } else {
                WHITE
            }
            Logger.recordOutput("Shooter", mechanism)
        }

        /** Shoot a ball at a given velocity and spin (in rad/s). */
        fun shoot(velocity: Double, spin: Double): Command =
            runEnd(
                {
                    val tangentialVelocity = spin * FLYWHEEL_SIDE_SEPERATION / 2.0

                    io.setSpeeds(
                        (velocity - tangentialVelocity) / FLYWHEEL_RADIUS,
                        (velocity + tangentialVelocity) / FLYWHEEL_RADIUS
                    )

                    // TODO: run rollers
                },
                {
                    io.setSpeeds(0.0, 0.0)

                    // TODO: stop rollers
                }
            )
    }

    object Pivot : Subsystem {
        private val io: PivotIO = when (Robot.model) {
            Robot.Model.SIMULATION -> PivotIOSim()
            Robot.Model.COMPETITION -> PivotIOKraken()
            Robot.Model.PRACTICE -> TODO()
        }
        private val inputs = PivotIO.Inputs()

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Pivot", inputs)

            armLigament.angle = inputs.position.degrees
            Logger.recordOutput("Shooter", mechanism)
        }

        fun pivotAndStop(goal: Rotation2d): Command =
            Commands.sequence(
                runOnce {
                    io.setSetpoint(
                        goal,
                        Rotation2d()
                    )
                },
                Commands.waitUntil {
                    (abs((goal - inputs.position).radians) < PIVOT_POSITION_TOLERANCE.radians)
                            && (abs(inputs.velocity.radians) < PIVOT_VELOCITY_TOLERANCE.radians)
                }
            )

        fun followMotionProfile(positionProfile: () -> Rotation2d, velocityProfile: () -> Rotation2d): Command =
            run {
                io.setSetpoint(positionProfile(), velocityProfile())
            }
    }

    // Register the two subsystems which together form the shooter.
    fun register() {
        Flywheels.register()
        Pivot.register()
    }

    private val mechanism = Mechanism2d(3.0, 3.0, BLACK)
    private val mechanismRoot = mechanism.getRoot("Shooter", 0.5, 0.5)
    private val armLigament = mechanismRoot.append(
        MechanismLigament2d(
            "Arm", 2.0, 0.0, 10.0, WHITE,
        )
    )
    private val flywheelLigament = armLigament.append(
        MechanismLigament2d(
            "Flywheel", 0.25, 0.0, 5.0, BLUE
        )
    )
}

internal val PIVOT_POSITION_TOLERANCE = Rotation2d.fromDegrees(2.0)
internal val PIVOT_VELOCITY_TOLERANCE = Rotation2d.fromDegrees(2.0)

internal val FLYWHEEL_RADIUS = Units.inchesToMeters(1.5)
internal val FLYWHEEL_SIDE_SEPERATION = Units.inchesToMeters(9.0)
