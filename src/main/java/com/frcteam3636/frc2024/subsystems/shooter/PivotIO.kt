package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.math.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DigitalInput
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

        var currentLeft: Double = 0.0
        var currentRight: Double = 0.0

        var rotorDistanceLeft: Double = 0.0
        var rotorVelocityLeft: Double = 0.0

        var rotorDistanceRight: Double = 0.0
        var rotorVelocityRight: Double = 0.0

        var leftLimitUnpressed = false

        override fun toLog(table: org.littletonrobotics.junction.LogTable) {
            table.put("Left Limit Unpressed", leftLimitUnpressed)
            table.put("Position", position)
            table.put("Velocity", velocity)
            table.put("Acceleration", acceleration)
            table.put("Current Left", currentLeft)
            table.put("Current Right", currentRight)
            table.put("Rotor Distance Left (rotations)", rotorDistanceLeft)
            table.put("Rotor Velocity Left", rotorVelocityLeft)
            table.put("Rotor Distance Right (rotations)", rotorDistanceRight)
            table.put("Rotor Velocity Right", rotorVelocityRight)
        }

        override fun fromLog(table: org.littletonrobotics.junction.LogTable) {
            leftLimitUnpressed = table.get("Left Limit Unpressed", leftLimitUnpressed)
            position = table.get("Position", position)[0]
            velocity = table.get("Velocity", velocity)[0]
            acceleration = table.get("Acceleration", acceleration)[0]
            currentLeft = table.get("Current Left", currentLeft)
            currentRight = table.get("Current Right", currentRight)
            rotorDistanceLeft = table.get("Rotor Distance Left (rotations)", rotorDistanceLeft)
            rotorVelocityLeft = table.get("Rotor Velocity Left", rotorVelocityLeft)
            rotorDistanceRight = table.get("Rotor Distance Right (rotations)", rotorDistanceRight)
            rotorVelocityRight = table.get("Rotor Velocity Right", rotorVelocityRight)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun pivotToAndStop(position: Rotation2d)
    fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d)

    fun driveCurrent(current: Double) {}
}

class PivotIOKraken : PivotIO {
    private val leftMotor = TalonFX(CTREMotorControllerId.LeftPivotMotor)
    private val rightMotor = TalonFX(CTREMotorControllerId.RightPivotMotor)
    private val leftLimitUnpressed = DigitalInput(2)

    init {
        val config = TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = PID_GAINS
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

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        leftMotor.configurator.apply(config)
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        rightMotor.configurator.apply(config)
    }

    override fun updateInputs(inputs: PivotIO.Inputs) {
        if (!leftLimitUnpressed.get()) {
            rightMotor.setPosition(LIMIT_SWITCH_POSITION)
            leftMotor.setPosition(-LIMIT_SWITCH_POSITION)
        }

        inputs.leftLimitUnpressed = leftLimitUnpressed.get()
        inputs.position = Rotation2d.fromRotations(leftMotor.position.value)
        inputs.velocity = Rotation2d.fromRotations(leftMotor.velocity.value)
        inputs.acceleration = Rotation2d.fromRotations(leftMotor.acceleration.value)

        //sysid shit
        inputs.currentLeft = leftMotor.torqueCurrent.value
        inputs.currentRight = rightMotor.torqueCurrent.value
        inputs.rotorDistanceLeft = Units.rotationsToRadians(leftMotor.rotorPosition.value)
        inputs.rotorVelocityLeft = Units.rotationsToRadians(leftMotor.rotorVelocity.value)
        inputs.rotorDistanceRight = Units.rotationsToRadians(rightMotor.rotorPosition.value)
        inputs.rotorVelocityRight = Units.rotationsToRadians(rightMotor.rotorVelocity.value)
    }

    override fun pivotToAndStop(position: Rotation2d) {
        val leftRequest = MotionMagicTorqueCurrentFOC(position.rotations)
        leftMotor.setControl(leftRequest)
        val rightRequest = MotionMagicTorqueCurrentFOC(position.rotations)
        rightMotor.setControl(rightRequest)

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", 0.0)
    }

    override fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d) {
        val leftControl = PositionTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations + LEFT_ZERO_OFFSET
            Velocity = velocity.rotations
        }
        leftMotor.setControl(leftControl)
        val rightControl = PositionTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations + RIGHT_ZERO_OFFSET
            Velocity = velocity.rotations
        }
        rightMotor.setControl(rightControl)

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", velocity)
    }

    override fun driveCurrent(current: Double) {
        leftMotor.setControl(TorqueCurrentFOC(current))
        rightMotor.setControl(TorqueCurrentFOC(current))
    }

    internal companion object Constants {
        val GEAR_RATIO = 120.0

        val PID_GAINS = PIDGains(0.0,0.0,0.0)
        val FF_GAINS = MotorFFGains(2.0, 0.0 ,0.0)
        val GRAVITY_GAIN = 2.5

        val PROFILE_VELOCITY = TAU
        val PROFILE_ACCELERATION = TAU
        val PROFILE_JERK = 10 * TAU

        const val LIMIT_SWITCH_POSITION = 0.52

        const val LEFT_ZERO_OFFSET = -0.496
        const val RIGHT_ZERO_OFFSET = 0.506
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