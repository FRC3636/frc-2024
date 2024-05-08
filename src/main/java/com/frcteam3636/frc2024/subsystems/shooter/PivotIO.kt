package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.TalonFXStatusProvider
import com.frcteam3636.frc2024.utils.math.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.math.abs
import kotlin.math.sign

interface PivotIO: TalonFXStatusProvider {
    class Inputs : LoggableInputs {
        /** The pitch of the pivot relative to the chassis. */
        var rightPosition: Rotation2d = Rotation2d()
        var leftPosition: Rotation2d = Rotation2d()

        var uncorrectedEncoderPosition = Rotation2d()
        var absoluteEncoderPosition: Rotation2d = Rotation2d()

        /** The angular velocity of the pivot. */
        var leftVelocity: Rotation2d = Rotation2d()
        var rightVelocity = Rotation2d()

        /** The angular acceleration of the pivot. */
        var acceleration: Rotation2d = Rotation2d()

        var voltageLeft: Double = 0.0
        var voltageRight: Double = 0.0

        var rotorDistanceLeft: Double = 0.0
        var rotorVelocityLeft: Double = 0.0

        var rotorDistanceRight: Double = 0.0
        var rotorVelocityRight: Double = 0.0
        var leftLimitSwitchUnpressed: Boolean = false

        override fun toLog(table: org.littletonrobotics.junction.LogTable) {
            table.put("Uncorrected Absolute Encoder Position", uncorrectedEncoderPosition)
            table.put("Absolute Encoder Position", absoluteEncoderPosition)
            table.put("Left Position", leftPosition)
            table.put("Right Position", rightPosition)
            table.put("Left Limit Switch Unpressed", leftLimitSwitchUnpressed)
            table.put("Right Position", rightPosition)
            table.put("Right Velocity", rightVelocity)
            table.put("Acceleration", acceleration)
            table.put("Voltage Left", voltageLeft)
            table.put("Voltage Right", voltageRight)
            table.put("Rotor Distance Left (rotations)", rotorDistanceLeft)
            table.put("Rotor Velocity Left", rotorVelocityLeft)
            table.put("Rotor Distance Right (rotations)", rotorDistanceRight)
            table.put("Rotor Velocity Right", rotorVelocityRight)
        }

        override fun fromLog(table: org.littletonrobotics.junction.LogTable) {
            uncorrectedEncoderPosition = table.get("Uncorrected Absolute Encoder Position", uncorrectedEncoderPosition)[0]
            absoluteEncoderPosition = table.get("Absolute Encoder Position", absoluteEncoderPosition)[0]
            leftLimitSwitchUnpressed = table.get("Left Limit Switch Unpressed", leftLimitSwitchUnpressed)
            absoluteEncoderPosition = table.get("Absolute Encoder Position", absoluteEncoderPosition)[0]
            leftPosition = table.get("Left Position", leftPosition)[0]
            rightPosition = table.get("Right Position", rightPosition)[0]
            leftVelocity = table.get("Left Velocity", leftVelocity)[0]
            rightVelocity = table.get("Right Velocity", rightVelocity)[0]
            acceleration = table.get("Acceleration", acceleration)[0]
            voltageLeft = table.get("Voltage Left", voltageLeft)
            voltageRight = table.get("Voltage Right", voltageRight)
            rotorDistanceLeft = table.get("Rotor Distance Left (rotations)", rotorDistanceLeft)
            rotorVelocityLeft = table.get("Rotor Velocity Left", rotorVelocityLeft)
            rotorDistanceRight = table.get("Rotor Distance Right (rotations)", rotorDistanceRight)
            rotorVelocityRight = table.get("Rotor Velocity Right", rotorVelocityRight)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun pivotToAndStop(position: Rotation2d)
    fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d)

    fun driveVoltage(volts: Double) {}
    fun setBrakeMode(enabled: Boolean) {}

    fun driveVelocity(velocity: Measure<Velocity<Angle>>) {}

    fun setPivotPosition(newPosition: Rotation2d) {}
}

class PivotIOKraken : PivotIO {
    private val leftMotor = TalonFX(CTREMotorControllerId.LeftPivotMotor)

    private val rightMotor = TalonFX(CTREMotorControllerId.RightPivotMotor)

    private val absoluteEncoder = DutyCycleEncoder(DigitalInput(2)).apply {
        distancePerRotation = SENSOR_TO_PIVOT_RATIO
    }
    private val rawAbsoluteEncoderPosition
        get() = Rotation2d.fromRotations(-absoluteEncoder.absolutePosition)

    init {
        val config = TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }

            Feedback.apply {
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
        leftMotor.configurator.apply(
            config
        )

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        rightMotor.configurator.apply(config)

        leftMotor.setPosition(LIMIT_SWITCH_OFFSET.rotations)
        rightMotor.setPosition(LIMIT_SWITCH_OFFSET.rotations)
    }

    private val leftLimitSwitchUnpressed = DigitalInput(1)

    override fun updateInputs(inputs: PivotIO.Inputs) {
        inputs.leftLimitSwitchUnpressed = leftLimitSwitchUnpressed.get()

        inputs.uncorrectedEncoderPosition = this.rawAbsoluteEncoderPosition
        inputs.absoluteEncoderPosition = Rotation2d(inputs.uncorrectedEncoderPosition.radians + ABSOLUTE_ENCODER_OFFSET.radians)

        inputs.leftPosition = Rotation2d.fromRotations(leftMotor.position.value)
        inputs.leftVelocity = Rotation2d.fromRotations(leftMotor.velocity.value)

        inputs.rightPosition = Rotation2d.fromRotations(rightMotor.position.value)
        inputs.rightVelocity = Rotation2d.fromRotations(rightMotor.velocity.value)

        inputs.acceleration = Rotation2d.fromRotations(rightMotor.acceleration.value)

        inputs.voltageLeft = leftMotor.motorVoltage.value
        inputs.voltageRight = rightMotor.motorVoltage.value
        inputs.rotorDistanceLeft = Units.rotationsToRadians(leftMotor.rotorPosition.value)
        inputs.rotorVelocityLeft = Units.rotationsToRadians(leftMotor.rotorVelocity.value)
        inputs.rotorDistanceRight = Units.rotationsToRadians(rightMotor.rotorPosition.value)
        inputs.rotorVelocityRight = Units.rotationsToRadians(rightMotor.rotorVelocity.value)
    }

    override fun pivotToAndStop(position: Rotation2d) {
        pivotToAndMove(position, Rotation2d())

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", 0.0)
    }

    override fun setPivotPosition(newPosition: Rotation2d) {

        leftMotor.setPosition(newPosition.rotations)
        rightMotor.setPosition(newPosition.rotations)
    }

    override fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d) {
        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)

        val leftControl = PositionTorqueCurrentFOC(0.0).apply {
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

    override fun setBrakeMode(enabled: Boolean) {
        leftMotor.setNeutralMode(
            if (enabled) {
                NeutralModeValue.Brake
            } else {
                NeutralModeValue.Coast
            }
        )
        rightMotor.setNeutralMode(
            if (enabled) {
                NeutralModeValue.Brake
            } else {
                NeutralModeValue.Coast
            }
        )
    }

    override fun driveVoltage(volts: Double) {
        leftMotor.setControl(VoltageOut(volts))
        rightMotor.setControl(VoltageOut(volts))
    }

    override fun driveVelocity(velocity: Measure<Velocity<Angle>>) {
        val control = VelocityTorqueCurrentFOC(velocity.`in`(edu.wpi.first.units.Units.RotationsPerSecond)).apply {
            Slot = 0
        }
        leftMotor.setControl(control)
        rightMotor.setControl(control)
    }

    internal companion object Constants {
        val GEAR_RATIO = 51.2
        val SENSOR_TO_PIVOT_RATIO = 1.0
        val TORQUE_CONSTANT = 0.5255

//        val PID_GAINS = PIDGains(120.0, 0.0, 100.0)
//        val FF_GAINS = MotorFFGains(7.8, 0.0, 0.0)

        //gains solved for analytically
        val PID_GAINS = PIDGains(1500.00594, 0.0, 125.0)
        val FF_GAINS = MotorFFGains(2.5, 0.0, 0.0)
        val GRAVITY_GAIN = 11.5

        val PROFILE_VELOCITY = 70.0
        val PROFILE_ACCELERATION = 50.0
        val PROFILE_JERK = 80.0

        const val LEFT_ZERO_OFFSET = -0.496
        const val RIGHT_ZERO_OFFSET = 0.38

        val LIMIT_SWITCH_OFFSET = Rotation2d.fromDegrees(-27.0)
        val ABSOLUTE_ENCODER_OFFSET = Rotation2d.fromDegrees(9.13) + LIMIT_SWITCH_OFFSET
    }

    override val talonCANStatuses = listOf(leftMotor.version, rightMotor.version)
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
        inputs.leftPosition = Rotation2d(state.position)
        inputs.leftVelocity = Rotation2d(state.velocity)
        inputs.rightPosition = Rotation2d(state.position)
        inputs.rightVelocity = Rotation2d(state.velocity)
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

    override val talonCANStatuses: List<StatusSignal<*>> = emptyList()
}
