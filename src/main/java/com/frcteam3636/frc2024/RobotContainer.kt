package com.frcteam3636.frc2024

import com.frcteam3636.frc2024.subsystems.intake.Intake
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
    private val controller = CommandXboxController(0)
    private val joystickLeft = Joystick(0)
    private val joystickRight = Joystick(1)

    init {
        configureBindings()
//        Shooter.register()
//        Drivetrain.register()
        Intake.register()
    }

    /** Use this method to define your `trigger->command` mappings. */
    private fun configureBindings() {
//        controller.x().whileTrue(Shooter.shootCommand())
        controller.b().whileTrue(Intake.intakeCommand())
//        Drivetrain.defaultCommand =
//            Drivetrain.driveWithJoysticks(translationJoystick = joystickLeft, rotationJoystick = joystickRight)
    }

    fun getAutonomousCommand(): Command? {
        return null
    }
}
