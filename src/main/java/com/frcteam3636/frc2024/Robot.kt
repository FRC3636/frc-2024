package com.frcteam3636.frc2024

import com.ctre.phoenix6.hardware.TalonFX
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.subsystems.drivetrain.OrientationTarget
import com.frcteam3636.frc2024.subsystems.intake.Intake
import com.frcteam3636.frc2024.subsystems.shooter.PivotProfile
import com.frcteam3636.frc2024.subsystems.shooter.Shooter
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton
 * class), and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. This is written as an object rather than a class since there should only ever be a
 * single instance, and it cannot take any constructor arguments. This makes it a natural fit to be
 * an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also
 * update the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when
 * renaming the object or package, it will get changed everywhere.)
 */
object Robot : LoggedRobot() {
    private val controller = CommandXboxController(2)
    private val joystickLeft = Joystick(0)
    private val joystickRight = Joystick(1)
    private var target: Target = Target.SPEAKER

    override fun robotInit() {
        // Report the use of the Kotlin Language for "FRC Usage Report" statistics
        HAL.report(
            tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version
        )

        if (isReal()) {
            Logger.addDataReceiver(WPILOGWriter("/U")) // Log to a USB stick
            Logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
            PowerDistribution(
                1, PowerDistribution.ModuleType.kRev
            ) // Enables power distribution logging
        } else {
            var logPath: String? = null
            try {
                logPath = LogFileUtil.findReplayLog() // Pull the replay log from AdvantageScope (or
                // prompt the user)
            } catch (_: java.util.NoSuchElementException) {
            }

            if (logPath == null) {
                // No replay log, so perform physics simulation
                Logger.addDataReceiver(NT4Publisher())
            } else {
                // Replay log exists, so replay data
                setUseTiming(false) // Run as fast as possible
                Logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
                Logger.addDataReceiver(
                    WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
                ) // Save outputs to a new log
            }
        }
        Logger.start() // Start logging! No more data receivers, replay sources, or metadata values
        // may be added.

        // initialize and register our subsystems
        Shooter.register()
        Drivetrain.register()
        Intake.register()

        TalonFX(0) // init phoenix diagnostics server

        // Configure our button and joystick bindings
        configureBindings()
    }

    private fun configureBindings() {
        Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks(
            translationJoystick = joystickLeft, rotationJoystick = joystickRight
        )


        Shooter.Pivot.defaultCommand = Shooter.Pivot.followMotionProfile(Target.STOWED.profile)

        controller.y().onTrue(InstantCommand({ target = Target.SPEAKER }))
        controller.a().onTrue( InstantCommand({ target = Target.AMP }))
        controller.leftTrigger().whileTrue(Shooter.Pivot.followMotionProfile(target.profile))

        controller.rightBumper().whileTrue(
            Commands.deadline(
                Commands.sequence(
                    Intake.intakeCommand().until { Shooter.Pivot.isPointingTowards(Target.STOWED.profile.position()) },
                    Intake.indexCommand()
                ),
                Shooter.Flywheels.intake()
            )
        )

        controller.x().onTrue(
            Shooter.Amp.pivotTo(Rotation2d.fromDegrees(170.0))
        ).onFalse(
            Shooter.Amp.stow()
        )

        Trigger(joystickRight::getTrigger).whileTrue(
            Commands.either(
                Shooter.Flywheels.shoot(40.0, 0.0),
                Shooter.Flywheels.shoot(2.5, 0.0)
            ) { target == Target.SPEAKER }
        )

        //Drive if triggered joystickLeft input

        JoystickButton(joystickLeft, 7).onTrue(
            InstantCommand({
                Drivetrain.defaultCommand = Drivetrain.driveWithJoystickPointingTowards(
                    joystickLeft, OrientationTarget.Speaker.position
                )
            })
        ).onFalse(
            InstantCommand({
                Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks(
                    translationJoystick = joystickLeft, rotationJoystick = joystickRight
                )
            })
        )

        JoystickButton(joystickLeft, 8).onTrue(
            InstantCommand({
                Drivetrain.zeroGyro()
                println("Gyro zeroed")
            })
        )
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        // TODO: start autonomous command
    }

    override fun teleopInit() {
//        Shooter.Amp.stow().schedule()
        // TODO: cancel autonomous command
    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    // A model of robot, depending on where we're deployed to.
    enum class Model {
        SIMULATION, PRACTICE, COMPETITION,
    }

    enum class Target(val profile: PivotProfile) {
        SPEAKER(
            PivotProfile(
                { Rotation2d.fromDegrees(95.0) },
                { Rotation2d() }
            )
        ),
        AMP(
            PivotProfile(
                { Rotation2d.fromDegrees(95.0) },
                { Rotation2d() }
            )
        ),
        STOWED(
            PivotProfile(
                { Rotation2d.fromDegrees(95.0) },
                { Rotation2d() }
            )
        )
    }

    // The model of this robot.
    val model: Model = if (RobotBase.isSimulation()) {
        Model.SIMULATION
    } else {
        when (val key = Preferences.getString("Model", "competition")) {
            "competition" -> Model.COMPETITION
            "practice" -> Model.PRACTICE
            else -> throw Exception("invalid model found in preferences: $key")
        }
    }
}
