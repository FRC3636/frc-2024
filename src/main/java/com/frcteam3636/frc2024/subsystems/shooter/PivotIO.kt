package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
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
        var leftPosition: Rotation2d = Rotation2d()

        /** The angular velocity of the pivot. */
        var velocity: Rotation2d = Rotation2d()

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
            table.put("Left Position", leftPosition)
            table.put("Left Limit Switch Unpressed", leftLimitSwitchUnpressed)
            table.put("Position", position)
            table.put("Velocity", velocity)
            table.put("Acceleration", acceleration)
            table.put("Voltage Left", voltageLeft)
            table.put("Voltage Right", voltageRight)
            table.put("Rotor Distance Left (rotations)", rotorDistanceLeft)
            table.put("Rotor Velocity Left", rotorVelocityLeft)
            table.put("Rotor Distance Right (rotations)", rotorDistanceRight)
            table.put("Rotor Velocity Right", rotorVelocityRight)
        }

        override fun fromLog(table: org.littletonrobotics.junction.LogTable) {

            leftLimitSwitchUnpressed = table.get("Left Limit Switch Unpressed", leftLimitSwitchUnpressed)
            leftPosition = table.get("Left Position", position)[0]
            position = table.get("Position", position)[0]
            velocity = table.get("Velocity", velocity)[0]
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
    fun holdPosition()

    fun driveVoltage(volts: Double) {}
    fun setBrakeMode(enabled: Boolean) {}

    fun resetPivotToHardStop() {}
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
        resetPivotToHardStop()
    }

    private val leftLimitSwitchUnpressed = DigitalInput(1)

    override fun updateInputs(inputs: PivotIO.Inputs) {

        if(!leftLimitSwitchUnpressed.get()){
            resetPivotToHardStop()
        }

        inputs.leftLimitSwitchUnpressed = leftLimitSwitchUnpressed.get()
        inputs.position = Rotation2d.fromRotations(rightMotor.position.value)
        inputs.leftPosition = Rotation2d.fromRotations(leftMotor.position.value)
        inputs.velocity = Rotation2d.fromRotations(rightMotor.velocity.value)
        inputs.acceleration = Rotation2d.fromRotations(rightMotor.acceleration.value)

        //sysid shit
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

    override fun resetPivotToHardStop() {
        leftMotor.setPosition(-LIMIT_SWITCH_OFFSET.rotations)
        rightMotor.setPosition(-LIMIT_SWITCH_OFFSET.rotations)
    }



    override fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d) {
        val leftControl = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations
//            Velocity = velocity.rotations

        }
        leftMotor.setControl(leftControl)
        val rightControl = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations
//            Velocity = velocity.rotations
        }
        rightMotor.setControl(rightControl)

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", 0.0)
    }

    override fun setBrakeMode(enabled: Boolean) {
        leftMotor.setNeutralMode(if (enabled) { NeutralModeValue.Brake } else { NeutralModeValue.Coast })
        rightMotor.setNeutralMode(if (enabled) { NeutralModeValue.Brake } else { NeutralModeValue.Coast })
    }

    override fun holdPosition() {
        val request = PositionTorqueCurrentFOC(leftMotor.position.value).apply {
            Slot = 0
        }
        leftMotor.setControl(request)
        rightMotor.setControl(request)

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", request.Position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", 0.0)
    }

    override fun driveVoltage(volts: Double) {
        leftMotor.setVoltage(volts)
        rightMotor.setVoltage(volts)
    }

    internal companion object Constants {
        val GEAR_RATIO = 40.0

        val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
//        val PID_GAINS = PIDGains()
        val FF_GAINS = MotorFFGains(7.0, 10.0, 0.0)
        val GRAVITY_GAIN = 13.0

        val PROFILE_VELOCITY = TAU / 2
        val PROFILE_ACCELERATION = TAU / 2
        val PROFILE_JERK = 10 * TAU

        const val LEFT_ZERO_OFFSET = -0.496
        const val RIGHT_ZERO_OFFSET = 0.38

        val LIMIT_SWITCH_OFFSET = Rotation2d.fromDegrees(27.0)
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

    override fun holdPosition() {
        TODO("Not yet implemented")
    }
}