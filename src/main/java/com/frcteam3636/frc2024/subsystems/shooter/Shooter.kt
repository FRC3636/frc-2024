package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.frcteam3636.frc2024.utils.math.PIDController
import com.frcteam3636.frc2024.utils.math.PIDGains
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
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
        ShooterIOReal()
    } else {
        TODO()
    }

    private val pidController = PIDController(PIDGains(0.1, 0.0, 0.0))
    private val rateLimiter = SlewRateLimiter(1.0)

    val inputs = ShooterIO.ShooterIOInputs()

    val tab = Shuffleboard.getTab("Shooter")
    val shouldSpin = tab.add("Should Spin", true).withWidget(BuiltInWidgets.kToggleSwitch).entry

    val motionMagicTorqueCurrentFOCRequest = MotionMagicTorqueCurrentFOC(0.0)

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

    fun startPivotingTo(setpoint: Rotation2d): Command = runOnce {
        io.setPivotControlRequest(
            motionMagicTorqueCurrentFOCRequest.withPosition(setpoint.rotations)
        )
    }



    fun pivotTo(setpoint: Rotation2d): Command =
        startPivotingTo(setpoint).andThen(WaitUntilCommand { inputs.atSetpoint })

    /**
     *translations in field space*
     * @param position the position of the robot
     * @param setpoint the position of the target
     */
    fun aimAtStatic(setpoint: Translation3d, position: Translation2d): Command =
        startPivotingTo(getAngleTo(setpoint, position))

    fun intakeCommand(): Command =
        startEnd({ io.intake(1.0) }, { io.intake(0.0) })


    private fun getAngleTo(target: Translation3d, position: Translation2d): Rotation2d {
        val distance = Translation2d(target.x, target.y).minus(position).norm
        val angle = atan(target.z / distance)
        return Rotation2d(angle)
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
    Trap1(Translation3d()),
    Trap2(Translation3d()),
    Trap3(Translation3d()),
}
