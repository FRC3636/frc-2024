package com.frcteam3636.frc2024

import com.frcteam3636.frc2024.subsystems.climber.Climber
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.subsystems.intake.Intake
import com.frcteam3636.frc2024.subsystems.shooter.SPEAKER_POSE
import com.frcteam3636.frc2024.subsystems.shooter.Shooter
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
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
    private val joystickDev = Joystick(3)
    private var autoChooser = SendableChooser<String>()

    private val brakeModeToggle = DigitalInput(5)

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
        Logger.start() // Start logging! No more data receivers, replay sources, or metadata values may be added.

        // initialize and register our subsystems
        Shooter.register()
        Drivetrain.register()
        Intake.register()
        Climber.register()

        // Configure our button and joystick bindings
        configureBindings()

        // Configure the autonomous command
        NamedCommands.registerCommand(
            "revAim",
            Commands.parallel(
                Shooter.Pivot.followMotionProfile(Shooter.Pivot.Target.AIM),
                Shooter.Flywheels.rev(590.0, 0.0)
            )
        )
        NamedCommands.registerCommand("stowIntakeRevAim",
            Commands.sequence(
                Commands.parallel(
                    Shooter.Pivot.pivotAndStop(Shooter.Pivot.Target.STOWED.profile.position()),
                    doIntakeSequence()
                ),
                Commands.parallel(
                    Shooter.Pivot.followMotionProfile(Shooter.Pivot.Target.AIM),
                    Shooter.Flywheels.rev(590.0, 0.0)
                )
            ))

        NamedCommands.registerCommand(
            "shootWhenReady",
            Commands.sequence(
                Commands.waitUntil(Shooter.Pivot.isReadyToShoot.and(Shooter.Flywheels.atDesiredVelocity)),
                Shooter.Feeder.feed().withTimeout(0.1)
            ))


        autoChooser.addOption("Middle 2 Piece", "Middle 2 Piece")
        autoChooser.addOption("Amp 2 Piece", "Left 2 Piece")
        autoChooser.addOption("Amp 3 Piece", "Left 3 Piece")
        autoChooser.addOption("Open Speaker Side 2 Piece", "Right 2 Piece")
        SmartDashboard.putData("Auto selector", autoChooser)
    }


    private fun configureBindings() {
        // Cartesian driving
        Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks(joystickLeft, joystickRight)

        // Polar driving
        Trigger(joystickLeft::getTrigger)
            .whileTrue(
                Drivetrain.driveWithJoystickPointingTowards(
                    joystickLeft,
                    SPEAKER_POSE.toTranslation2d()
                )
            )

        // Control the climber
        controller.povUp().debounce(0.15).whileTrue(Climber.setClimber(0.5))
        controller.povDown().debounce(0.15).whileTrue(Climber.setClimber(-0.5))
        controller.povRight().onTrue(Climber.knockIntake())


        // Follow a motion profile to the selected pivot target
        controller.leftTrigger()
            .debounce(0.1)
            .whileTrue(Shooter.Pivot.followMotionProfile(null))
            .onFalse(Shooter.Pivot.followMotionProfile(Shooter.Pivot.Target.STOWED))

        // Select a target for the pivot
        controller.a().onTrue(Shooter.Pivot.setTarget(Shooter.Pivot.Target.AMP))
        controller.b().onTrue(Shooter.Pivot.setTarget(Shooter.Pivot.Target.AIM))
        controller.y().onTrue(Shooter.Pivot.setTarget(Shooter.Pivot.Target.PODIUM))

        // Intake
        controller.rightBumper()
            .debounce(0.150)
            .whileTrue(
                doIntakeSequence()
            )

        // Outtake
        controller.leftBumper()
            .whileTrue(
                Commands.parallel(
                    Intake.outtake(),
                    Shooter.Flywheels.outtake(),
                    Shooter.Feeder.outtake()
                ).finallyDo(Runnable {
                    Note.state = Note.State.NONE
                })
            )


        // Manually feed through the shooter.
        controller.x().whileTrue(
            Shooter.Feeder.feed()
        )

        Shooter.Amp.defaultCommand = Shooter.Amp.pivotTo(Rotation2d(0.0))

        // Shoot a note.
        Trigger(joystickRight::getTrigger)
            .whileTrue(
                Commands.parallel(
                    Commands.either(
                        Shooter.Flywheels.rev(590.0, 0.0),
                        Shooter.Flywheels.rev(2.5, 0.0)
                    ) { Shooter.Pivot.target != Shooter.Pivot.Target.AMP },
                    Commands.sequence(
                        Commands.waitUntil(Shooter.Flywheels.atDesiredVelocity),
                        Shooter.Feeder.feed().withTimeout(0.4).beforeStarting({
                            if (Note.state == Note.State.SHOOTER) {
                                Note.state = Note.State.NONE
                            }
                        }),
                    )
                )
            )

        JoystickButton(joystickLeft, 8).onTrue(Commands.runOnce({ Drivetrain.zeroGyro() }))

        JoystickButton(joystickLeft, 9).debounce(0.15).whileTrue(Shooter.Pivot.pivotAndStop(Rotation2d(-25.5)))

        // Switch pivot brake mode on and off while disabled.
        Trigger(brakeModeToggle::get)
            .toggleOnTrue(Shooter.Pivot.setBrakeMode(true).ignoringDisable(true))
            .toggleOnFalse(Shooter.Pivot.setBrakeMode(false).ignoringDisable(true))
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        autoCommand = AutoBuilder.buildAuto(autoChooser.selected)
        autoCommand!!.schedule()
    }

    override fun teleopInit() {
        autoCommand?.cancel()
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

private fun doIntakeSequence(): Command =
    Commands.sequence(
        Intake.intake(),
        Commands.runOnce({ Note.state = Note.State.HANDOFF }),
        Commands.waitUntil(Shooter.Pivot::isStowed),
        Commands.race(
            Commands.parallel(
                Intake.index(),
                Shooter.Feeder.intake(),
                Shooter.Flywheels.intake(),
            ),
            Commands.sequence(
                //spinning up
                Commands.waitUntil { Shooter.Flywheels.aboveIntakeThreshold },
                //reached velocity setpoint
                Commands.waitUntil { !Shooter.Flywheels.aboveIntakeThreshold },
                //contacted note
                Commands.waitUntil { Shooter.Flywheels.aboveIntakeThreshold },
                //note stowed
                Commands.waitUntil { !Shooter.Flywheels.aboveIntakeThreshold },
                Commands.runOnce({ Note.state = Note.State.SHOOTER })
            )
        )
    )

object Note {
    enum class State(val index: Long) {
        NONE(0),
        HANDOFF(1),
        SHOOTER(2)
    }

    var state: State = State.SHOOTER
        set(value) {
            field = value
            Logger.recordOutput("Note State", value.name)
            rgbPublisher.setInteger(value.index)
        }

    private val rgbPublisher = NetworkTableInstance.getDefault().getTopic("RGB/Note State").genericPublish("int")
}


