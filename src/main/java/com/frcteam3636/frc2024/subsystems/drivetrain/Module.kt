package com.frcteam3636.frc2024.subsystems.drivetrain

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.frcteam3636.frc2024.*
import com.frcteam3636.frc2024.utils.math.*
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.SparkAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import kotlin.math.roundToInt

interface SwerveModule {
    // The current "state" of the swerve module.
    //
    // This is essentially the velocity of the wheel,
    // and includes both the speed and the angle
    // in which the module is currently traveling.
    val state: SwerveModuleState

    // The desired state of the module.
    //
    // This is the wheel velocity that we're trying to get to.
    var desiredState: SwerveModuleState

    // The measured position of the module.
    //
    // This is a vector with direction equal to the current angle of the module,
    // and magnitude equal to the total signed distance traveled by the wheel.
    val position: SwerveModulePosition

    fun periodic() {}
}

class MAXSwerveModule(
    private val drivingMotor: DrivingMotor, turningId: REVMotorControllerId, private val chassisAngle: Rotation2d
) : SwerveModule {
    private val turningSpark = CANSparkMax(turningId, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()

        idleMode = CANSparkBase.IdleMode.kBrake
        setSmartCurrentLimit(TURNING_CURRENT_LIMIT.roundToInt())
    }

    // whereas the turning encoder must be absolute so that
    // we know where the wheel is pointing
    private val turningEncoder = turningSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).apply {
        // invert the encoder because the output shaft rotates opposite to the motor itself
        inverted = true

        // convert native units of rotations and RPM to radians and radians per second
        // tau = 2 * pi = circumference / radius
        positionConversionFactor = TAU
        velocityConversionFactor = TAU / 60
    }


    private val turningPIDController = turningSpark.pidController.apply {
        setFeedbackDevice(turningEncoder)
        pidGains = TURNING_PID_GAINS

        // enable PID wrapping so that the controller will go across zero to the setpoint
        positionPIDWrappingEnabled = true
        positionPIDWrappingMinInput = 0.0
        positionPIDWrappingMaxInput = TAU
    }

    init {
        turningSpark.burnFlash()
    }

    override val state: SwerveModuleState
        get() = SwerveModuleState(
            drivingMotor.velocity, Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
        )

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
            drivingMotor.position, Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
        )

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, -chassisAngle)
        get() = SwerveModuleState(field.speedMetersPerSecond, field.angle + chassisAngle)
        set(value) {
            val corrected = SwerveModuleState(value.speedMetersPerSecond, value.angle - chassisAngle)
            // optimize the state to avoid rotating more than 90 degrees
            val optimized = SwerveModuleState.optimize(
                corrected, Rotation2d.fromRadians(turningEncoder.position)
            )

            drivingMotor.velocity = optimized.speedMetersPerSecond

            turningPIDController.setReference(
                optimized.angle.radians, CANSparkBase.ControlType.kPosition
            )

            field = optimized
        }

}

interface DrivingMotor {
    val position: Double
    var velocity: Double
}

class DrivingTalon(id: CTREMotorControllerId) : DrivingMotor {

    private val inner = TalonFX(id).apply {
        configurator.apply(Slot0Configs().apply {
            pidGains = DRIVING_PID_GAINS_TALON
            motorFFGains = DRIVING_FF_GAINS_TALON
        })
        configurator.apply(
            CurrentLimitsConfigs().apply {
                SupplyCurrentLimit = DRIVING_CURRENT_LIMIT
            }
        )
    }

    override val position: Double
        get() = inner.position.value * DRIVING_GEAR_RATIO_TALON * WHEEL_CIRCUMFERENCE

    override var velocity: Double
        get() = inner.velocity.value
        set(value) {
            inner.setControl(VelocityTorqueCurrentFOC(value / DRIVING_GEAR_RATIO_TALON / WHEEL_CIRCUMFERENCE).withSlot(0))
        }
}

class DrivingSparkMAX(id: REVMotorControllerId) : DrivingMotor {
    private val inner = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()

        inverted = true

        idleMode = CANSparkBase.IdleMode.kBrake
        setSmartCurrentLimit(DRIVING_CURRENT_LIMIT.roundToInt())
    }

    init {
        inner.encoder.apply {
            // convert native units of rotations and RPM to meters and meters per second
            positionConversionFactor = DRIVING_GEAR_RATIO_NEO * WHEEL_CIRCUMFERENCE
            velocityConversionFactor = DRIVING_GEAR_RATIO_NEO * WHEEL_CIRCUMFERENCE / 60
        }

        inner.pidController.apply {
            setFeedbackDevice(inner.encoder)

            pidGains = DRIVING_PID_GAINS_NEO
            ff = DRIVING_FF_GAINS_NEO.v
        }
    }

    override val position: Double
        get() = inner.encoder.position

    override var velocity: Double
        get() = inner.encoder.velocity
        set(value) {
            inner.set(value)
        }
}

class SimSwerveModule : SwerveModule {

    // TODO: figure out what the moment of inertia actually is and if it even matters
    private val turningMotor = DCMotorSim(DCMotor.getNeo550(1), TAU, 0.0001)
    private val drivingMotor = DCMotorSim(DCMotor.getKrakenX60(1), 6.75, 0.0025)

    private val drivingFeedforward = SimpleMotorFeedforward(MotorFFGains(v = 3.33))
    private val drivingFeedback = PIDController(PIDGains(0.06))

    private val turningFeedback = PIDController(PIDGains(p = 2.0)).apply { enableContinuousInput(0.0, TAU) }

    override val state: SwerveModuleState
        get() = SwerveModuleState(
            drivingMotor.angularVelocityRadPerSec * WHEEL_RADIUS,
            Rotation2d.fromRadians(turningMotor.angularPositionRad)
        )

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())
        set(value) {
            field = SwerveModuleState.optimize(value, state.angle)
        }

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
            drivingMotor.angularPositionRad * WHEEL_RADIUS, Rotation2d.fromRadians(turningMotor.angularPositionRad)
        )

    override fun periodic() {
        turningMotor.update(Robot.period)
        drivingMotor.update(Robot.period)

        // Set the new input voltages
        turningMotor.setInputVoltage(
            turningFeedback.calculate(state.angle.radians, desiredState.angle.radians)
        )
        drivingMotor.setInputVoltage(
            drivingFeedforward.calculate(desiredState.speedMetersPerSecond) + drivingFeedback.calculate(
                state.speedMetersPerSecond, desiredState.speedMetersPerSecond
            )
        )
    }
}

// take the known wheel diameter, divide it by two to get the radius, then get the
// circumference
internal val WHEEL_RADIUS = Units.inchesToMeters(3.0) / 2
internal val WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * TAU

internal const val NEO_FREE_SPEED_RPM = 5676.0
internal const val NEO_FREE_SPEED_RPS: Double = NEO_FREE_SPEED_RPM / 60

internal val DRIVING_GEAR_RATIO_TALON = 1.0 / 3.56
internal val DRIVING_GEAR_RATIO_NEO = 0.0

internal val DRIVING_PID_GAINS_TALON: PIDGains = PIDGains(4.0, 0.0, 0.1)
internal val DRIVING_PID_GAINS_NEO: PIDGains = PIDGains(0.04, 0.0, 0.0)
internal val DRIVING_FF_GAINS_TALON: MotorFFGains = MotorFFGains(5.75, 0.0)
internal val DRIVING_FF_GAINS_NEO: MotorFFGains =
    MotorFFGains(0.0, 1 / NEO_FREE_SPEED_RPS, 0.0) // TODO: ensure this is right

internal val TURNING_PID_GAINS: PIDGains = PIDGains(1.7, 0.0, 0.125)
internal val DRIVING_CURRENT_LIMIT = 30.0
internal val TURNING_CURRENT_LIMIT = 20.0
