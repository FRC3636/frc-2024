package com.frcteam3636.frc2024

import com.ctre.phoenix6.hardware.TalonFX
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.subsystems.drivetrain.OrientationTarget
import com.frcteam3636.frc2024.subsystems.intake.Intake
import com.frcteam3636.frc2024.subsystems.shooter.Shooter
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

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

    private var autoCommand: Command? = null

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

        // Configure the autonomous command
        NamedCommands.registerCommand("intake", Intake.intakeCommand())
        NamedCommands.registerCommand("pivot", Shooter.Pivot.pivotAndStop(Rotation2d(Units.degreesToRadians(100.0))))
        NamedCommands.registerCommand("zeropivot", Shooter.Pivot.pivotAndStop(Rotation2d(0.0)))
        NamedCommands.registerCommand("shoot", Shooter.Flywheels.shoot(15.0, 0.0).withTimeout(3.0))

        autoCommand = AutoBuilder.buildAuto("Middle 2 Piece")
    }

    private fun configureBindings() {
        Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks(
            translationJoystick = joystickLeft, rotationJoystick = joystickRight
        )


        controller.a().onTrue(Shooter.Pivot.pivotAndStop(Rotation2d.fromDegrees(135.0)))

        controller.b().onTrue(Shooter.Flywheels.shoot(40.0, 0.0))

        controller.y().onTrue(InstantCommand({Shooter.Pivot.zeroPivot()}))

        controller.leftBumper().onTrue(Shooter.Flywheels.index())

        controller.rightBumper().whileTrue(Commands.sequence(
            Commands.parallel(
                Intake.intakeCommand(),
                Shooter.Pivot.pivotAndStop(Rotation2d.fromDegrees(-27.0))
            ),
            Commands.parallel(
                Shooter.Flywheels.intake(),
                Intake.indexCommand()
            )))

        controller.rightTrigger().whileTrue(Commands.parallel(
            Shooter.Flywheels.intake(),
            Intake.indexCommand()
        ))


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

        JoystickButton(
            joystickLeft, 2
        ).whileTrue(
            Shooter.Pivot.followMotionProfile({ Rotation2d(PI / 4 + sin(Timer.getFPGATimestamp()) / 2) },
                { Rotation2d(cos(Timer.getFPGATimestamp()) / 2) })
        )
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        autoCommand!!.schedule()
    }

    override fun teleopInit() {
        autoCommand!!.cancel()
    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    // A model of robot, depending on where we're deployed to.
    enum class Model {
        SIMULATION, PRACTICE, COMPETITION,
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
