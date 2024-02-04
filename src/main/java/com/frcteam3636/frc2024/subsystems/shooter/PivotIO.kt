package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.signals.GravityTypeValue
import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.math.*
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

interface PivotIO {
    class Inputs : LoggableInputs {
        /** The pitch of the pivot relative to the chassis. */
        var position: Rotation2d = Rotation2d()

        /** The angular velocity of the pivot. */
        var velocity: Rotation2d = Rotation2d()

        /** The angular acceleration of the pivot. */
        var acceleration: Rotation2d = Rotation2d()

        override fun toLog(table: org.littletonrobotics.junction.LogTable) {
            table.put("Position", position)
            table.put("Velocity", velocity)
            table.put("Acceleration", acceleration)
        }

        override fun fromLog(table: org.littletonrobotics.junction.LogTable) {
            position = table.get("Position", position)[0]
            velocity = table.get("Velocity", velocity)[0]
            acceleration = table.get("Acceleration", acceleration)[0]
        }
    }

    fun updateInputs(inputs: Inputs)

    fun pivotToAndStop(position: Rotation2d)
    fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d)

    fun driveVoltage(volts: Double) {}
}

class PivotIOKraken : PivotIO {
    private val leftMotor = TalonFX(CTREMotorControllerId.LeftPivotMotor)
    private val rightMotor = TalonFX(CTREMotorControllerId.RightPivotMotor)

    init {
        val config = TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = PIDGains()
                motorFFGains = FF_GAINS
                GravityType = GravityTypeValue.Arm_Cosine
                kG = GRAVITY_GAIN
            }

            Feedback.apply {
                SensorToMechanismRatio = GEAR_RATIO
                FeedbackRotorOffset = 0.0
            }

            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_VELOCITY
                MotionMagicAcceleration = PROFILE_ACCELERATION
                MotionMagicJerk = PROFILE_JERK
            }
        }

        leftMotor.configurator.apply(config)
        rightMotor.configurator.apply(config)
    }

    override fun updateInputs(inputs: PivotIO.Inputs) {
        inputs.position = Rotation2d.fromRotations(leftMotor.position.value * GEAR_RATIO)
        inputs.velocity = Rotation2d.fromRotations(leftMotor.velocity.value * GEAR_RATIO)
        inputs.acceleration = Rotation2d.fromRotations(leftMotor.acceleration.value * GEAR_RATIO)
    }

    override fun pivotToAndStop(position: Rotation2d) {
        val request = MotionMagicTorqueCurrentFOC(position.rotations)
        leftMotor.setControl(request)
        rightMotor.setControl(request)

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", 0.0)
    }

    override fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d) {
        val control = PositionTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations
            Velocity = velocity.rotations
        }

        leftMotor.setControl(control)
        rightMotor.setControl(control)

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", velocity)
    }

    override fun driveVoltage(volts: Double) {
        leftMotor.setVoltage(volts)
        rightMotor.setVoltage(volts)
    }

    internal companion object Constants {
        val GEAR_RATIO = 1.0 / 90.0

        val PID_GAINS = PIDGains()
        val FF_GAINS = MotorFFGains()
        val GRAVITY_GAIN = 0.0

        val PROFILE_VELOCITY = TAU
        val PROFILE_ACCELERATION = TAU
        val PROFILE_JERK = 10 * TAU
    }
}

class PivotIONeo : PivotIO {
    private val profile: TrapezoidProfile = TrapezoidProfile(PROFILE_CONSTRAINTS)


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
        follow(leftPivot)
    }

    private val pid = leftPivot.pidController.apply {
        p = 0.0
        i = 0.0
        d = 0.0
        ff = 0.0
        iZone = 0.0
        setOutputRange(0.0, 0.0)

        setSmartMotionMaxVelocity(0.0, 0)
        setSmartMotionMinOutputVelocity(0.0, 0)
        setSmartMotionMaxAccel(0.0, 0)
        setSmartMotionAllowedClosedLoopError(1.0, 0)
        setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0)
    }

    init {
        leftPivot.burnFlash()
        rightPivot.burnFlash()
    }

    private val pivotPID = PIDController(PIDGains(0.0, 0.0, 0.0))
    private val pivotFeedForward = ArmFeedforward(0.0, 0.0, 0.0, 0.0)


    override fun updateInputs(inputs: PivotIO.Inputs) {
        inputs.position = Rotation2d(leftPivot.encoder.position * PIVOT_GEAR_RATIO)
        inputs.velocity = Rotation2d(leftPivot.encoder.velocity * PIVOT_GEAR_RATIO)
        //inputs.acceleration = Rotation2d(leftPivot.encoder.velocity * PIVOT_GEAR_RATIO)
    }

    override fun pivotToAndStop(position: Rotation2d) {
        pivotToAndMove(position, Rotation2d())
    }

    override fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d) {
        pid.setSmartMotionMinOutputVelocity(velocity.rotations, 0)
        pid.setReference(
            position.radians,
            CANSparkBase.ControlType.kSmartMotion
        )
    }

    override fun driveVoltage(volts: Double) {
        leftPivot.setVoltage(volts)
        rightPivot.setVoltage(volts)
    }

    internal companion object Constants {
        val PROFILE_CONSTRAINTS = TrapezoidProfile.Constraints(0.0, 0.0)

        const val PIVOT_GEAR_RATIO = 1 / 90.0
    }
}

class PivotIOSim : PivotIO {
    private val profile = TrapezoidProfile(
        TrapezoidProfile.Constraints(
            PivotIOKraken.PROFILE_VELOCITY,
            PivotIOKraken.PROFILE_ACCELERATION,
        )
    )
    private val profileTimer = Timer().apply { start() }

    private var start = TrapezoidProfile.State()
    private var goal = TrapezoidProfile.State()

    override fun updateInputs(inputs: PivotIO.Inputs) {
        val state = profile.calculate(profileTimer.get(), start, goal)
        inputs.position = Rotation2d(state.position)
        inputs.velocity = Rotation2d(state.velocity)
    }

    override fun pivotToAndStop(position: Rotation2d) {
        pivotToAndMove(position, Rotation2d())
    }

    override fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d) {
        start = profile.calculate(profileTimer.get(), start, goal)
        goal = TrapezoidProfile.State(position.radians, velocity.radians)
        profileTimer.reset()

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", 0.0)
    }
}