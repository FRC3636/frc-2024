package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.math.*
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.RobotController
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

    fun driveVoltage(volts: Double) {}

    fun resetPivotToHardStop() {}
}

class PivotIOKraken : PivotIO {
    private val leftMotor = TalonFX(CTREMotorControllerId.LeftPivotMotor)
    private val rightMotor = TalonFX(CTREMotorControllerId.RightPivotMotor)

    init {
        val config = TalonFXConfiguration().apply{
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
        val leftControl = PositionTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations
            Velocity = velocity.rotations

        }
        leftMotor.setControl(leftControl)
        val rightControl = PositionTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.rotations
            Velocity = velocity.rotations
        }
        rightMotor.setControl(rightControl)

        Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
        Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", velocity)
    }

    override fun driveVoltage(volts: Double) {
        leftMotor.setVoltage(volts)
        rightMotor.setVoltage(volts)
    }

    internal companion object Constants {
        val GEAR_RATIO = 40.0

        val PID_GAINS = PIDGains(175.0, 0.0, 0.0)
        val FF_GAINS = MotorFFGains(2.8, 0.0, 0.0)
        val GRAVITY_GAIN = 13.0

        val PROFILE_VELOCITY = TAU
        val PROFILE_ACCELERATION = TAU
        val PROFILE_JERK = 10 * TAU

        const val LEFT_ZERO_OFFSET = -0.496
        const val RIGHT_ZERO_OFFSET = 0.38

        val LIMIT_SWITCH_OFFSET = Rotation2d.fromDegrees(27.0)
    }
}

class PivotIONeo : PivotIO {
    private val profile: TrapezoidProfile = TrapezoidProfile(PROFILE_CONSTRAINTS)


    private val leftMotor = CANSparkMax(
        REVMotorControllerId.LeftPivotMotor, CANSparkLowLevel.MotorType.kBrushless
    ).apply {
        restoreFactoryDefaults()
        inverted = true
        encoder.positionConversionFactor = TAU * PIVOT_GEAR_RATIO
        encoder.velocityConversionFactor = TAU * PIVOT_GEAR_RATIO / 60.0
//        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * PIVOT_GEAR_RATIO
//        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * PIVOT_GEAR_RATIO / 60
    }

    private val rightMotor = CANSparkMax(
        REVMotorControllerId.RightPivotMotor, CANSparkLowLevel.MotorType.kBrushless
    ).apply {
        restoreFactoryDefaults()
        inverted = true
        encoder.positionConversionFactor = TAU / PIVOT_GEAR_RATIO
        encoder.velocityConversionFactor = TAU / PIVOT_GEAR_RATIO / 60.0
//        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * PIVOT_GEAR_RATIO
//        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * PIVOT_GEAR_RATIO / 60
    }

    private val pid = leftMotor.pidController.apply {
        p = 0.0
        i = 0.0
        d = 0.0
        ff = 0.0
        iZone = 0.0
        setOutputRange(0.0, 0.0)

//        setSmartMotionMaxVelocity(0.0, 0)
//        setSmartMotionMinOutputVelocity(0.0, 0)
//        setSmartMotionMaxAccel(0.0, 0)
//        setSmartMotionAllowedClosedLoopError(1.0, 0)
//        setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0)
    }

    init {
        leftMotor.burnFlash()
        rightMotor.burnFlash()
    }

    private val pivotPID = PIDController(PIDGains(0.0, 0.0, 0.0))
    private val pivotFeedForward = ArmFeedforward(0.0, 0.0, 0.0, 0.0)


    override fun updateInputs(inputs: PivotIO.Inputs) {


        inputs.position = Rotation2d.fromRotations(leftMotor.encoder.position)
        inputs.velocity = Rotation2d.fromRotations(leftMotor.encoder.velocity)


        //sysid shit
        //velocity in rps cause talonfx uses those units and consistency ykyk
        inputs.voltageLeft = leftMotor.appliedOutput * RobotController.getBatteryVoltage()
        inputs.voltageRight = rightMotor.appliedOutput * RobotController.getBatteryVoltage()
        inputs.rotorDistanceLeft = leftMotor.encoder.position
        inputs.rotorVelocityLeft = leftMotor.encoder.velocity
        inputs.rotorDistanceRight = rightMotor.encoder.position
        inputs.rotorVelocityRight = rightMotor.encoder.velocity
        //inputs.acceleration = Rotation2d(leftPivot.encoder.velocity * PIVOT_GEAR_RATIO)
    }

    override fun pivotToAndStop(position: Rotation2d) {
        pivotToAndMove(position, Rotation2d())
    }

    override fun pivotToAndMove(position: Rotation2d, velocity: Rotation2d) {
        pid.setSmartMotionMinOutputVelocity(velocity.rotations, 0)
        pid.setReference(
            position.radians, CANSparkBase.ControlType.kSmartMotion
        )
    }

    override fun driveVoltage(volts: Double) {
        leftMotor.setVoltage(volts)
        rightMotor.setVoltage(volts)
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