package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.math.MotorFFGains
import com.frcteam3636.frc2024.utils.math.PIDController
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.frcteam3636.frc2024.utils.math.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

object Shooter {

    val pivotIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(), SysIdRoutine.Mechanism(
            Pivot::setvoltage, Pivot::getState, Pivot
        )
    )

    val flywheelIORoutine = SysIdRoutine(
        SysIdRoutine.Config(), SysIdRoutine.Mechanism(Flywheels::setVoltage, Flywheels::getState, Flywheels)
    )

    object Flywheels : Subsystem {
        private val io: FlywheelIO = when (Robot.model) {
            Robot.Model.SIMULATION -> FlywheelIOSim()
            Robot.Model.COMPETITION, Robot.Model.PRACTICE -> FlywheelIOReal()
        }
        private val inputs = FlywheelIO.Inputs()

        private var setpointLeft = RadiansPerSecond.zero()
        private var setpointRight = RadiansPerSecond.zero()
        private val pidControllerLeft = PIDController(FLYWHEEL_PID_GAINS)
        private val pidControllerRight = PIDController(FLYWHEEL_PID_GAINS)
        private val ffController = SimpleMotorFeedforward(FLYWHEEL_FF_GAINS)

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Flywheels", inputs)

            flywheelLigament.color = if (inputs.leftSpeed > RotationsPerSecond.of(1.0)) {
                BLUE
            } else {
                WHITE
            }
            Logger.recordOutput("Shooter", mechanism)

//            io.setVoltage(
//                Volts.of(
//                    ffController.calculate(setpointLeft.magnitude()) + pidControllerLeft.calculate(
//                        inputs.leftSpeed.magnitude(),
//                        setpointLeft.magnitude()
//                    )
//                ),
//                Volts.of(
//                    ffController.calculate(setpointRight.magnitude()) + pidControllerRight.calculate(
//                        inputs.rightSpeed.magnitude(),
//                        setpointRight.magnitude()
//                    )
//                )
//            )
            Logger.recordOutput("Shooter/Flywheels/Left Setpoint", setpointLeft)
            Logger.recordOutput("Shooter/Flywheels/Right Setpoint", setpointRight)
        }

        /** Shoot a ball at a given velocity and spin (in rad/s). */
        fun shoot(velocity: Double, spin: Double): Command = runEnd({
            val tangentialVelocity = spin * FLYWHEEL_SIDE_SEPERATION / 2.0

            setpointLeft = RadiansPerSecond.of((velocity - tangentialVelocity) / FLYWHEEL_RADIUS)
            setpointRight = RadiansPerSecond.of((velocity + tangentialVelocity) / FLYWHEEL_RADIUS)

            // TODO: run rollers
        }, {
            setpointLeft = RadiansPerSecond.zero()
            setpointRight = RadiansPerSecond.zero()

            // TODO: stop rollers
        })

        fun intake(): Command {
            return runEnd({
                setpointLeft = RadiansPerSecond.of(1.0)
                setpointRight = RadiansPerSecond.of(1.0)
            }, {
                setpointLeft = RadiansPerSecond.zero()
                setpointRight = RadiansPerSecond.zero()
            })
        }

        fun setVoltage(volts: Measure<Voltage>) {
            io.setVoltage(volts, volts)
        }

        fun getState(log: SysIdRoutineLog) {
            log.motor("left-flywheels")
                .voltage(
                    Volts.of(inputs.leftVoltage)
                )
                .angularVelocity(
                    inputs.leftSpeed
                )
            log.motor("right-flywheels")
                .voltage(
                    Volts.of(inputs.rightVoltage)
                )
                .angularVelocity(
                    inputs.rightSpeed
                )
        }
    }

    object Pivot : Subsystem {
        private val io: PivotIO = when (Robot.model) {
            Robot.Model.SIMULATION -> PivotIOSim()
            Robot.Model.COMPETITION -> PivotIOKraken()
            Robot.Model.PRACTICE -> PivotIONeo()
        }
        private val inputs = PivotIO.Inputs()

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Pivot", inputs)

            armLigament.angle = inputs.position.degrees
            Logger.recordOutput("Shooter", mechanism)
        }

        fun pivotAndStop(goal: Rotation2d): Command = Commands.sequence(runOnce {
            io.pivotToAndStop(goal)
        }, Commands.waitUntil {
            (abs((goal - inputs.position).radians) < PIVOT_POSITION_TOLERANCE.radians)
                    && (abs(inputs.velocity.radians) < PIVOT_VELOCITY_TOLERANCE.radians)
        })

        fun getState(log: SysIdRoutineLog) {
            log.motor("pivot-left").voltage(
                Volts.of(inputs.voltageLeft)
            ).angularPosition(
                Radians.of(inputs.rotorDistanceLeft)
            ).angularVelocity(
                RadiansPerSecond.of(inputs.rotorVelocityLeft)
            )
            log.motor("pivot-right").voltage(
                Volts.of(inputs.voltageRight)
            ).angularPosition(
                Radians.of(inputs.rotorDistanceRight)
            ).angularVelocity(
                RadiansPerSecond.of(inputs.rotorVelocityRight)
            )
        }

        fun setvoltage(volts: Measure<Voltage>): Command = runOnce {
            io.driveVoltage(volts.magnitude())
        }

        fun followMotionProfile(positionProfile: () -> Rotation2d, velocityProfile: () -> Rotation2d): Command = run {
            io.pivotToAndMove(positionProfile(), velocityProfile())
        }

        enum class PositionPresets(position: Rotation2d) {
            Handoff(Rotation2d()), Amp(Rotation2d())
        }
    }

    // Register the two subsystems which together form the shooter.
    fun register() {
        Flywheels.register()
        Pivot.register()
    }

    private val mechanism = Mechanism2d(3.0, 3.0, BLACK)
    private val mechanismRoot = mechanism.getRoot("Shooter", 0.5, 0.5)
    private val armLigament = mechanismRoot.append(
        MechanismLigament2d(
            "Arm", 2.0, 0.0, 10.0, WHITE,
        )
    )
    private val flywheelLigament = armLigament.append(
        MechanismLigament2d(
            "Flywheel", 0.25, 0.0, 5.0, BLUE
        )
    )
}

internal val PIVOT_POSITION_TOLERANCE = Rotation2d.fromDegrees(2.0)
internal val PIVOT_VELOCITY_TOLERANCE = Rotation2d.fromDegrees(2.0)

internal val FLYWHEEL_RADIUS = Units.inchesToMeters(1.5)
internal val FLYWHEEL_SIDE_SEPERATION = Units.inchesToMeters(9.0)
internal val FLYWHEEL_PID_GAINS = PIDGains(0.05, 0.0, 0.0)
internal val FLYWHEEL_FF_GAINS = MotorFFGains()

internal val BLACK = Color8Bit("#0a0a0a")
internal val WHITE = Color8Bit("#ffffff")
internal val BLUE = Color8Bit("#1d48a3")
