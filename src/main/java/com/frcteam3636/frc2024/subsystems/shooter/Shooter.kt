package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.utils.math.PIDController
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.frcteam3636.frc2024.utils.math.dot
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.littletonrobotics.junction.Logger
import java.math.BigDecimal
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sqrt

object Shooter : Subsystem {

    const val ACCELERATION_PROFILE = 0.0
    const val VELOCITY_PROFILE = 0.0
    const val JERK_PROFILE = 0.0

    private val io: ShooterIO = if (RobotBase.isReal()) {
        ShooterIORealTalon()
    } else {
        TODO()
    }

    private val pidController = PIDController(PIDGains(0.1, 0.0, 0.0))
    private val rateLimiter = SlewRateLimiter(1.0)

    val inputs = ShooterIO.ShooterIOInputs()

    val tab = Shuffleboard.getTab("Shooter")
    val shouldSpin = tab.add("Should Spin", true).withWidget(BuiltInWidgets.kToggleSwitch).entry


    val targetVelocity = tab.add("Target Velocity", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(mapOf(Pair("min", 0.0), Pair("max", 6000.0))).entry

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)
    }

    fun shootCommand(): Command = run {
        io.shoot(
            rateLimiter.calculate(
                pidController.calculate(
                    inputs.leftSpeed.radians,
                    targetVelocity.getDouble(0.0)
                )
            ),
            shouldSpin.getBoolean(false)
        )
    }

    /**
     * Start pivoting to the setpoint, and arrive with velocity (angle / sec)
     *
     * @param position the position of the robot
     * @param targetPosition the target to aim at
     */
    fun aimAtAndTrack(position: Translation2d, targetPosition: TargetPosition): Command {

        val setpoint = getAngleTo(targetPosition.position, position)
        val distanceVector = targetPosition.position.toTranslation2d().minus(position)
        val targetHeight = targetPosition.position.z
        val distance = distanceVector.norm
        val normalizedDistanceVector = distanceVector.div(distance)
        val derivativeDistance =
            Translation2d(Drivetrain.chassisSpeeds.vxMetersPerSecond, Drivetrain.chassisSpeeds.vyMetersPerSecond)
                .dot(normalizedDistanceVector)

        return runOnce {
            io.setPivotPositionWithVelocity(
                setpoint,
                Rotation2d.fromRotations(
                    Shooter.getVelocityToTarget(
                        distance,
                        targetHeight
                    ).rotations * derivativeDistance
                )

            )
        }

    }

    /**
     * Start pivoting to the setpoint, command terminates instantly
     *
     * @param setpoint the position to pivot to
     */

    fun startPivotingTo(setpoint: Rotation2d): Command = runOnce {
        io.setPivotPosition(setpoint)
    }

    /**
     * Start pivoting to the setpoint, command terminates when the pivot arrives
     *
     * @param setpoint the position to pivot to
     */
    fun pivotTo(setpoint: Rotation2d): Command =
        startPivotingTo(setpoint).andThen(WaitUntilCommand { inputs.atSetpoint })

    /**
     * Start aiming at a target to arrive with 0 velocity.
     *
     * @param position the position of the robot
     * @param setpoint the position of the target
     */
    fun aimAtStatic(setpoint: TargetPosition, position: Translation2d): Command =
        startPivotingTo(getAngleTo(setpoint.position, position))

    fun intakeCommand(): Command =
        startEnd({ io.intake(1.0) }, { io.intake(0.0) })

    /**
     * Return the two dimensional angle to the target (x horizontal against z) based on the position of the robot
     *
     * @param target the position of the target
     * @param position the position of the robot
     */
    private fun getAngleTo(target: Translation3d, position: Translation2d): Rotation2d {
        val distance = Translation2d(target.x, target.y).minus(position).norm
        val angle = atan(target.z / distance)
        return Rotation2d(angle)
    }

    /**
     * Return the velocity, in radians per second,
     * for the pivot to track the correct angle to the target based on distance away from the target
     *
     * keep units of distance and height consistent or it won't work grr
     *
     * @param distance the distance from the target
     * @param targetHeight the height of the target
     */
    fun getVelocityToTarget(distance: Double, targetHeight: Double): Rotation2d {
        val radPerSec = -targetHeight / targetHeight.pow(2) + distance.pow(2)
        return Rotation2d(radPerSec)
    }

    /**
     * Return the acceleration, in radians per second squared,
     * for the pivot to track the correct angle to the target based on distance away from the target
     *
     * keep units of distance and height consistent or it won't work grr
     *
     * @param distance the distance from the target
     * @param targetHeight the height of the target
     */
    fun getAccelerationToTarget(distance: Double, targetHeight: Double): Rotation2d {
        val radPerSecSquared =
            (2 * targetHeight * distance) / (targetHeight.pow(2) + distance.pow(2)).pow(2)
        return Rotation2d(radPerSecSquared)
    }

}

//0 degrees = pivot pointing horizontally outwards
enum class PivotPosition(val position: Rotation2d) {
    Horizontal(Rotation2d(0.0)),
    Vertical(Rotation2d.fromDegrees(90.0)),
    Handoff(Rotation2d.fromDegrees(190.0)),
    Amps(Rotation2d.fromDegrees(80.0))
}

enum class TargetPosition(val position: Translation3d) {
    Speaker(Translation3d()),

    //this shit prolly never gonna happen lol
    Trap1(Translation3d()),
    Trap2(Translation3d()),
    Trap3(Translation3d()),
    Amp(Translation3d())
}
