package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.GravityTypeValue
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.math.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
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

    fun setSetpoint(position: Rotation2d, velocity: Rotation2d)
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

    override fun setSetpoint(position: Rotation2d, velocity: Rotation2d) {
        val request = MotionMagicTorqueCurrentFOC(position.rotations)
        leftMotor.setControl(request)
        rightMotor.setControl(request)
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

    override fun setSetpoint(position: Rotation2d, velocity: Rotation2d) {
        start = profile.calculate(profileTimer.get(), start, goal)
        goal = TrapezoidProfile.State(position.radians, velocity.radians)
        profileTimer.reset()

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", velocity)
    }
}