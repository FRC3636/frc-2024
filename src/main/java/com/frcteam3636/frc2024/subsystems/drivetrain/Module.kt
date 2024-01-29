package com.frcteam3636.frc2024.subsystems.drivetrain

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.SlotConfigs
import com.ctre.phoenix6.controls.VelocityVoltage
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
    drivingId: CTREMotorControllerId,
    turningId: REVMotorControllerId,
    private val chassisAngle: Rotation2d
) : SwerveModule {
    private val drivingTalon =
        TalonFX(drivingId).apply {
            configurator.apply(SlotConfigs().withMotorFFGains(DRIVING_FF_GAINS))
            configurator.apply(
                CurrentLimitsConfigs().withSupplyCurrentLimit(DRIVING_CURRENT_LIMIT)
            )
        }

    private val turningSpark =
        CANSparkMax(turningId, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()

            idleMode = CANSparkBase.IdleMode.kBrake
            setSmartCurrentLimit(TURNING_CURRENT_LIMIT.roundToInt())
        }

    // whereas the turning encoder must be absolute so that
    // we know where the wheel is pointing
    private val turningEncoder =
        turningSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).apply {
            // invert the encoder because the output shaft rotates opposite to the motor itself
            inverted = true

            // convert native units of rotations and RPM to radians and radians per second
            // tau = 2 * pi = circumference / radius
            positionConversionFactor = TAU
            velocityConversionFactor = TAU / 60
        }

    private val turningPIDController =
        turningSpark.pidController.apply {
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
        get() =
            SwerveModuleState(
                drivingTalon.velocity.value * DRIVING_MOTOR_TRAVEL_PER_REVOLUTION,
                Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
            )

    override val position: SwerveModulePosition
        get() =
            SwerveModulePosition(
                drivingTalon.position.value * DRIVING_MOTOR_TRAVEL_PER_REVOLUTION,
                Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
            )

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, -chassisAngle)
        get() = SwerveModuleState(field.speedMetersPerSecond, field.angle + chassisAngle)
        set(value) {
            val corrected =
                SwerveModuleState(value.speedMetersPerSecond, value.angle - chassisAngle)
            // optimize the state to avoid rotating more than 90 degrees
            val optimized =
                SwerveModuleState.optimize(
                    corrected,
                    Rotation2d.fromRadians(turningEncoder.position)
                )

            drivingTalon.setControl(
                VelocityVoltage(
                    optimized.speedMetersPerSecond /
                            DRIVING_MOTOR_TRAVEL_PER_REVOLUTION
                )
                    .withSlot(0)
            )

            turningPIDController.setReference(
                optimized.angle.radians,
                CANSparkBase.ControlType.kPosition
            )

            field = optimized
        }

    internal companion object Constants {
        // MAXSwerve can be configured with different pinion gears to make the module faster or
        // increase torque
        val DRIVING_MOTOR_PINION_TEETH = 14

        // The gear ratio between the motor and the wheel.
        // I.e. the wheel angle divided by the motor angle.
        // Motor Pinion : Motor Spur Gear = x :
        // Bevel Pinion : Wheel Bevel Gear = 15 : 45
        val DRIVING_MOTOR_TO_WHEEL_GEARING =
            (DRIVING_MOTOR_PINION_TEETH.toDouble() / 22.0) * (15.0 / 45.0)

        // take the known wheel diameter, divide it by two to get the radius, then get the
        // circumference
        val WHEEL_RADIUS = Units.inchesToMeters(3.0) / 2
        val WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * TAU

        // The distance travelled by one rotation of the driving motor.
        val DRIVING_MOTOR_TRAVEL_PER_REVOLUTION =
            WHEEL_CIRCUMFERENCE * DRIVING_MOTOR_TO_WHEEL_GEARING

        val DRIVING_PID_GAINS: PIDGains = PIDGains()
        val DRIVING_FF_GAINS: MotorFFGains = MotorFFGains()

        val TURNING_PID_GAINS: PIDGains = PIDGains()
        val DRIVING_CURRENT_LIMIT = 60.0
        val TURNING_CURRENT_LIMIT = 20.0
    }
}

class SimSwerveModule : SwerveModule {

    // TODO: figure out what the moment of inertia actually is and if it even matters
    private val turningMotor = DCMotorSim(DCMotor.getNeo550(1), TAU, 0.0001)
    private val drivingMotor = DCMotorSim(DCMotor.getKrakenX60(1), 6.75, 0.0025)

    private val drivingFeedforward = SimpleMotorFeedforward(MotorFFGains(v = 3.33))
    private val drivingFeedback = PIDController(PIDGains(0.06))

    private val turningFeedback =
        PIDController(PIDGains(p = 2.0)).apply { enableContinuousInput(0.0, TAU) }

    override val state: SwerveModuleState
        get() =
            SwerveModuleState(
                drivingMotor.angularVelocityRadPerSec * MAXSwerveModule.WHEEL_RADIUS,
                Rotation2d.fromRadians(turningMotor.angularPositionRad)
            )

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())
        set(value) {
            field = SwerveModuleState.optimize(value, state.angle)
        }

    override val position: SwerveModulePosition
        get() =
            SwerveModulePosition(
                drivingMotor.angularPositionRad * MAXSwerveModule.WHEEL_RADIUS,
                Rotation2d.fromRadians(turningMotor.angularPositionRad)
            )

    override fun periodic() {
        turningMotor.update(Robot.period)
        drivingMotor.update(Robot.period)

        // Set the new input voltages
        turningMotor.setInputVoltage(
            turningFeedback.calculate(state.angle.radians, desiredState.angle.radians)
        )
        drivingMotor.setInputVoltage(
            drivingFeedforward.calculate(desiredState.speedMetersPerSecond) +
                    drivingFeedback.calculate(
                        state.speedMetersPerSecond,
                        desiredState.speedMetersPerSecond
                    )
        )
    }
}
