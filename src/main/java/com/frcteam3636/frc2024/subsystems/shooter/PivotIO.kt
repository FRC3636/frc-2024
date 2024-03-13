package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.math.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

interface PivotIO {
    class Inputs : LoggableInputs {
        /** The pitch of the pivot relative to the chassis. */
        var position: Rotation2d = Rotation2d()

        /** The angular velocity of the pivot. */
        var velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RotationsPerSecond)

        /** The angular acceleration of the pivot. */
        var acceleration: MutableMeasure<Velocity<Velocity<Angle>>> = MutableMeasure.zero(RotationsPerSecond.per(Second))

        /** The torque-equivalent current applied to the motor. **/
        var currentLeft: MutableMeasure<Current> = MutableMeasure.zero(Amps)
        var currentRight: MutableMeasure<Current> = MutableMeasure.zero(Amps)

        override fun toLog(table: org.littletonrobotics.junction.LogTable) {
            table.put("Position", position)
            table.put("Velocity", velocity)
            table.put("Acceleration", acceleration)

            table.put("Current Left", currentLeft)
            table.put("Current Right", currentLeft)
        }

        override fun fromLog(table: org.littletonrobotics.junction.LogTable) {
            position = table.get("Position", position)[0]
            velocity = table.get("Velocity", velocity)
            acceleration = table.get("Acceleration", acceleration)
            currentLeft = table.get("Voltage Left", currentLeft)
            currentRight = table.get("Voltage Right", currentRight)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun pivotToAndStop(position: Rotation2d)
    fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d)

    fun setNeutralMode(mode: NeutralModeValue)

    // used for sysid
    fun driveVoltage(volts: Double)
}

class PivotIOKraken : PivotIO {
    private val leftMotor = TalonFX(CTREMotorControllerId.LeftPivotMotor)
    private val rightMotor = TalonFX(CTREMotorControllerId.RightPivotMotor)

    init {
        val config = TalonFXConfiguration().apply{
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }

            Feedback.apply{
                SensorToMechanismRatio = GEAR_RATIO
                FeedbackRotorOffset = 0.0
            }

            Slot0.apply {
                pidGains = PID_GAINS
                motorFFGains = FF_GAINS
                GravityType = GravityTypeValue.Arm_Cosine
                kG = GRAVITY_GAIN
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

    private val leftLimitSwitchUnpressed = DigitalInput(1)

    override fun updateInputs(inputs: PivotIO.Inputs) {
        inputs.position = Rotation2d.fromRotations(rightMotor.position.value)
        inputs.velocity.mut_setMagnitude(rightMotor.velocity.value)
        inputs.acceleration.mut_setMagnitude(rightMotor.acceleration.value)

        inputs.currentLeft.mut_setMagnitude(leftMotor.torqueCurrent.value)
        inputs.currentLeft.mut_setMagnitude(rightMotor.torqueCurrent.value)
    }

    override fun pivotToAndStop(position: Rotation2d) {
        pivotToAndMove(position, Rotation2d())

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", 0.0)
    }

    override fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d) {
        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        val leftControl = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations
        }
        leftMotor.setControl(leftControl)
        val rightControl = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations
        }
        rightMotor.setControl(rightControl)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", 0.0)
    }

    override fun setNeutralMode(mode: NeutralModeValue) {
        leftMotor.setNeutralMode(mode)
        rightMotor.setNeutralMode(mode)
    }

    override fun driveVoltage(volts: Double) {
        leftMotor.setControl(VoltageOut(volts))
        rightMotor.setControl(VoltageOut(volts))
    }

    internal companion object Constants {
        val GEAR_RATIO = 40.0

        val PID_GAINS = PIDGains(120.0, 0.0, 100.0)
        val FF_GAINS = MotorFFGains(7.8, 0.0, 0.0)
        val GRAVITY_GAIN = 10.0

        // TODO: these need to be tuned
        val PROFILE_VELOCITY = 4.0
        val PROFILE_ACCELERATION = 8.0
        val PROFILE_JERK = 20.0

        const val LEFT_ZERO_OFFSET = -0.496
        const val RIGHT_ZERO_OFFSET = 0.38
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
        inputs.position = Rotation2d.fromRotations(state.position)
        inputs.velocity.mut_setMagnitude(state.velocity)
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

    override fun setNeutralMode(mode: NeutralModeValue) {}

    override fun driveVoltage(volts: Double) {
        TODO()
    }
}