package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.math.MotorFFGains
import com.frcteam3636.frc2024.utils.math.PIDController
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.math.abs


interface ShooterIO {
    class ShooterIOInputs : LoggableInputs {
        var leftSpeed = Rotation2d()
        var rightSpeed = Rotation2d()
        var pivotPosition = Rotation2d()
        var pivotVelocity = Rotation2d()
        var pivotAcceleration = Rotation2d()
        var atSetpoint = true

        override fun toLog(table: LogTable) {
            table.put("At setpoint", atSetpoint)
            table.put("Left Speed", leftSpeed)
            table.put("Right Speed", rightSpeed)
            table.put("Position", pivotPosition)
            table.put("Velocity", pivotVelocity)
            table.put("Acceleration", pivotAcceleration)
        }

        override fun fromLog(table: LogTable) {
            leftSpeed = table.get("Left Speed", leftSpeed)[0]
            rightSpeed = table.get("Right Speed", rightSpeed)[0]
            pivotPosition = table.get("Position", pivotPosition)!![0]
            pivotVelocity = table.get("Velocity", pivotVelocity)!![0]
            pivotAcceleration = table.get("Acceleration", pivotAcceleration)!![0]
        }
    }

    /** Updates the set of loggable inputs. */
    fun updateInputs(inputs: ShooterIOInputs)

    /** Run the launcher flywheel at the specified percent speed. */
    fun shoot(speed: Double, spin: Boolean)

    /** Run the launcher flywheels in reverse to intake at the specified percent speed. */
    fun intake(speed: Double)

    fun setPivotPosition(setpoint: Rotation2d)

    fun setPivotPositionWithVelocity(setpoint: Rotation2d, velocity: Rotation2d)
}


class ShooterIOPractice : ShooterIO {
    private val profile: TrapezoidProfile = TrapezoidProfile(PROFILE_CONSTRAINTS)


    private val leftFlywheels = CANSparkMax(
        REVMotorControllerId.LeftShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless
    ).apply {
        inverted = true
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO / 60
    }

    private val rightFlywheels = CANSparkMax(
        REVMotorControllerId.RightShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless
    ).apply {
        inverted = true
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO / 60
    }

    private val leftPivot = CANSparkMax(
        REVMotorControllerId.LeftPivotMotor, CANSparkLowLevel.MotorType.kBrushless
    ).apply {
        inverted = true
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * PIVOT_GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * PIVOT_GEAR_RATIO / 60
    }

    private val rightPivot = CANSparkMax(
        REVMotorControllerId.RightPivotMotor, CANSparkLowLevel.MotorType.kBrushless
    ).apply {
        inverted = true
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * PIVOT_GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * PIVOT_GEAR_RATIO / 60
    }

    private val timer: Timer = Timer()

    private val pivotPID = PIDController(PIDGains(0.0, 0.0, 0.0))
    private val pivotFeedForward = ArmFeedforward(0.0, 0.0, 0.0, 0.0)


    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        inputs.leftSpeed = Rotation2d(leftFlywheels.encoder.velocity)
        inputs.rightSpeed = Rotation2d(rightFlywheels.encoder.velocity)
        inputs.pivotPosition = Rotation2d(leftPivot.encoder.position * PIVOT_GEAR_RATIO)
        inputs.pivotVelocity = Rotation2d(leftPivot.encoder.velocity * PIVOT_GEAR_RATIO)
        inputs.pivotAcceleration = Rotation2d(leftPivot.encoder.velocity * PIVOT_GEAR_RATIO)
    }

    override fun shoot(speed: Double, spin: Boolean) {
        if (spin) {
            leftFlywheels.set(speed)
            rightFlywheels.set(speed)
        } else {
            leftFlywheels.set(speed)
            rightFlywheels.set(speed * 0.75)
        }
    }

    override fun intake(speed: Double) {
        leftFlywheels.set(-speed)
        rightFlywheels.set(-speed)
    }

    override fun setPivotPosition(setpoint: Rotation2d) {
        setPivotPositionWithVelocity(setpoint, Rotation2d())
    }

    override fun setPivotPositionWithVelocity(setpoint: Rotation2d, velocity: Rotation2d) {
        val goalState = TrapezoidProfile.State(setpoint.radians, velocity.radians)
        timer.start()
        do {
            val current = TrapezoidProfile.State(
                Rotation2d(leftPivot.encoder.position * PIVOT_GEAR_RATIO).radians,
                Rotation2d(leftPivot.encoder.velocity * PIVOT_GEAR_RATIO / 60).radians
            )
            val goal = profile.calculate(timer.get(), current, goalState)
            val voltage = pivotFeedForward.calculate(current.position, goal.velocity) + pivotPID.calculate(
                current.position, goal.position
            )
            leftPivot.setVoltage(voltage)
            rightPivot.setVoltage(voltage)
        } while (!((abs(leftPivot.encoder.position - setpoint.radians) > 0.1) || abs(rightPivot.encoder.position - setpoint.radians) > 0.1))

        timer.stop()
        timer.reset()
    }

    internal companion object Constants {
        val PROFILE_CONSTRAINTS = TrapezoidProfile.Constraints(0.0, 0.0)

        const val PIVOT_GEAR_RATIO = 1 / 90.0
        const val FLYWHEEL_GEAR_RATIO = 1.0
    }
}


class ShooterIOComp : ShooterIO {
    private val flywheelLeft = CANSparkMax(
        REVMotorControllerId.LeftShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless
    ).apply {
        inverted = false
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO / 60
    }
    private val flywheelRight = CANSparkMax(
        REVMotorControllerId.RightShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless
    ).apply {
        inverted = true
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO / 60
    }

    private val pivotLeft = TalonFX(CTREMotorControllerId.LeftPivotMotor)
    private val pivotRight = TalonFX(CTREMotorControllerId.RightPivotMotor)

    init {
        val pivotConfig =
            TalonFXConfiguration().withFeedback(FeedbackConfigs().withSensorToMechanismRatio(PIVOT_GEAR_RATIO))
                .withMotionMagic(
                    MotionMagicConfigs().withMotionMagicAcceleration(
                        MOTION_MAGIC_ACCELERATION
                    ).withMotionMagicJerk(MOTION_MAGIC_JERK)
                )

        pivotLeft.configurator.apply(pivotConfig)
        pivotRight.configurator.apply(pivotConfig)
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        inputs.leftSpeed = Rotation2d(flywheelLeft.encoder.velocity)
        inputs.rightSpeed = Rotation2d(flywheelRight.encoder.velocity)

        inputs.pivotPosition = Rotation2d(pivotLeft.position.value * Constants.PIVOT_GEAR_RATIO)
        inputs.pivotVelocity = Rotation2d(pivotLeft.velocity.value * Constants.PIVOT_GEAR_RATIO)
        inputs.pivotAcceleration = Rotation2d(pivotLeft.acceleration.value * Constants.PIVOT_GEAR_RATIO)

        inputs.atSetpoint =
            pivotLeft.closedLoopError.value < Constants.MOTION_PROFILE_ERROR_THRESHOLD && pivotRight.closedLoopError.value < Constants.MOTION_PROFILE_ERROR_THRESHOLD
    }

    override fun shoot(speed: Double, spin: Boolean) {
        flywheelLeft.set(speed)
        Logger.recordOutput("Shooter/Left Power", speed)
        val rSpeed = speed * if (spin) {
            0.75
        } else {
            1.0
        }
        flywheelRight.set(rSpeed)
        Logger.recordOutput("Shooter/Right Power", rSpeed)
    }

    override fun setPivotPosition(setpoint: Rotation2d) {
        val control = MotionMagicTorqueCurrentFOC(0.0).withPosition(setpoint.rotations)
        pivotLeft.setControl(control)
        pivotRight.setControl(control)
    }

    override fun setPivotPositionWithVelocity(setpoint: Rotation2d, velocity: Rotation2d) {

        val control = PositionTorqueCurrentFOC(0.0).withSlot(0).apply {
            Position = setpoint.rotations
            Velocity = velocity.rotations
        }

        pivotLeft.setControl(control)
        pivotRight.setControl(control)
    }

    override fun intake(speed: Double) {
        flywheelLeft.set(speed)
        flywheelRight.set(speed)
        Logger.recordOutput("Shooter/Left Power", speed)
        Logger.recordOutput("Shooter/Right Power", speed)
    }

    internal companion object Constants {
        const val FLYWHEEL_GEAR_RATIO = 1.0
        const val PIVOT_GEAR_RATIO = 1 / 90.0

        val PIVOT_FF_GAINS = MotorFFGains()
        val PIVOT_PID_GAINS = PIDGains()
        const val kG = 0.0

        const val MOTION_MAGIC_ACCELERATION = 0.0
        const val MOTION_MAGIC_JERK = 4000.0

        const val MOTION_PROFILE_ERROR_THRESHOLD =
            2.0 // i have no idea what units these are in, we'll have to just fuck around and
        // find out
    }
}
