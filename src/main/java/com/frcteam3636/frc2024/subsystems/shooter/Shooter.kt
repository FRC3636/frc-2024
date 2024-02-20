package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.BLACK
import com.frcteam3636.frc2024.BLUE
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.WHITE
import com.frcteam3636.frc2024.utils.math.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

object Shooter {


    object Flywheels : Subsystem {
        private val io: FlywheelIO = when (Robot.model) {
            Robot.Model.SIMULATION -> FlywheelIOSim()
            Robot.Model.COMPETITION, Robot.Model.PRACTICE -> FlywheelIOReal()
        }
        private val inputs = FlywheelIO.Inputs()

        private var pidfControlEnabled = true
        private var setpointLeft = RadiansPerSecond.zero()
        private var setpointRight = RadiansPerSecond.zero()
        private val pidControllerLeft = PIDController(FLYWHEEL_PID_GAINS) // TODO: set tolerance
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

            if (pidfControlEnabled) {
                io.setFlywheelVoltage(
                    Volts.of(
                        ffController.calculate(setpointLeft.`in`(RadiansPerSecond)) + pidControllerLeft.calculate(
                            inputs.leftSpeed.`in`(RadiansPerSecond),
                            setpointLeft.`in`(RadiansPerSecond)
                        )
                    ),
                    Volts.of(
                        ffController.calculate(setpointRight.`in`(RadiansPerSecond)) + pidControllerRight.calculate(
                            inputs.rightSpeed.`in`(RadiansPerSecond),
                            setpointRight.`in`(RadiansPerSecond)
                        )
                    )
                )
                Logger.recordOutput("Shooter/Flywheels/Left Setpoint", setpointLeft)
                Logger.recordOutput("Shooter/Flywheels/Right Setpoint", setpointRight)
            }
            Logger.recordOutput("Shooter/Flywheels/PIDF Control Enabled", pidfControlEnabled)
        }

        /** Shoot a ball at a given velocity and spin (in rad/s). */
        fun shoot(velocity: Double, spin: Double): Command =
            Commands.parallel(
                // start the flywheels
                runEnd({
                    pidfControlEnabled = true

                    val tangentialVelocity = spin * FLYWHEEL_SIDE_SEPERATION / 2.0

                    setpointLeft = RadiansPerSecond.of((velocity - tangentialVelocity) / FLYWHEEL_RADIUS)
                    setpointRight = RadiansPerSecond.of((velocity + tangentialVelocity) / FLYWHEEL_RADIUS)
                }, {
                    setpointLeft = RadiansPerSecond.zero()
                    setpointRight = RadiansPerSecond.zero()
                }),
                Commands.sequence(
                    // wait for the flywheels to get up to speed
                    Commands.waitUntil { pidControllerLeft.atSetpoint() && pidControllerRight.atSetpoint() },
                    // run the indexer.
                    // note that not taking `this` as a requirement here is a hack, but ultimately fine because
                    // we're the subsystem
                    Commands.runEnd({
                        io.setIndexerVoltage(Volts.of(-10.0))
                    }, {
                        io.setIndexerVoltage(Volts.zero())
                    })
                )
            )

        fun index(): Command =
            runEnd({
                io.setIndexerVoltage(Volts.of(-10.0))
            }, {
                io.setIndexerVoltage(Volts.of(0.0))
            })

        fun intake(): Command =
            runEnd({
                pidfControlEnabled = true

                setpointLeft = RadiansPerSecond.of(-50.0)
                setpointRight = RadiansPerSecond.of(-50.0)
                io.setIndexerVoltage(Volts.of(6.78))
            }, {
                setpointLeft = RadiansPerSecond.zero()
                setpointRight = RadiansPerSecond.zero()
                io.setIndexerVoltage(Volts.of(0.0))
            })

        val sysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(), SysIdRoutine.Mechanism(
                {
                    pidfControlEnabled = false
                    io.setFlywheelVoltage(it, it)
                },
                {
                    it.motor("left-flywheels")
                        .voltage(Volts.of(inputs.leftVoltage))
                        .angularPosition(inputs.leftPos)
                        .angularVelocity(inputs.leftSpeed)
                    it.motor("right-flywheels")
                        .voltage(Volts.of(inputs.rightVoltage))
                        .angularPosition(inputs.rightPos)
                        .angularVelocity(inputs.rightSpeed)
                },
                this
            )
        )

        fun doDynamicSysId(direction: SysIdRoutine.Direction): Command =
            sysIdRoutine.dynamic(direction).beforeStarting(runOnce { pidfControlEnabled = false })

        fun doQuasistaticSysId(direction: SysIdRoutine.Direction): Command =
            sysIdRoutine.quasistatic(direction).beforeStarting(runOnce { pidfControlEnabled = false })
    }

    object Pivot : Subsystem {
        private val io: PivotIO = when (Robot.model) {
            Robot.Model.SIMULATION -> PivotIOSim()
            Robot.Model.COMPETITION -> PivotIOKraken()
            Robot.Model.PRACTICE -> PivotIONeo()
        }
        private val inputs = PivotIO.Inputs()
        private var pivotOffset: Double = 0.0
        private val leftLimitSwitchUnpressed = DigitalInput(2)
        private val rightLimitSwitchUnpressed = DigitalInput(3)

        override fun periodic() {
//            if(!leftLimitSwitchUnpressed.get() || !rightLimitSwitchUnpressed.get()){
//                pivotOffset = -inputs.position.radians
//            }
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Pivot", inputs)

            armLigament.angle = inputs.position.degrees
            Logger.recordOutput("Shooter", mechanism)
        }

        fun pivotAndStop(goal: Rotation2d): Command = Commands.parallel(
            runOnce { io.pivotToAndStop(goal) },
            Commands.waitUntil {
                (abs((goal - inputs.position).radians) < PIVOT_POSITION_TOLERANCE.radians) && (abs(inputs.velocity.radians) < PIVOT_VELOCITY_TOLERANCE.radians)
            }
        )

        fun followMotionProfile(positionProfile: () -> Rotation2d, velocityProfile: () -> Rotation2d): Command = run {
            io.pivotToAndMove(positionProfile(), velocityProfile())
        }


        val sysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(Volts.of(1.0).per(Seconds.of(2.0)), null, null, null),
            SysIdRoutine.Mechanism(
                {
                    io.driveVoltage(it.baseUnitMagnitude())
                },
                {
                    it.motor("pivot-left")
                        .voltage(Volts.of(inputs.voltageLeft))
                        .angularPosition(Radians.of(inputs.rotorDistanceLeft))
                        .angularVelocity(RadiansPerSecond.of(inputs.rotorVelocityLeft))
                    it.motor("pivot-right")
                        .voltage(Volts.of(inputs.voltageRight))
                        .angularPosition(Radians.of(inputs.rotorDistanceRight))
                        .angularVelocity(RadiansPerSecond.of(inputs.rotorVelocityRight))
                },
                this
            )
        )

        fun doDynamicSysId(direction: SysIdRoutine.Direction): Command =
            sysIdRoutine.dynamic(direction)
                .until {
                    if (direction == SysIdRoutine.Direction.kForward) {
                        inputs.position.rotations > 0.4
                    } else {
                        inputs.position.rotations < -0.3
                    }
                }
                .andThen(runOnce { io.driveVoltage(0.0) })

        fun doQuasistaticSysId(direction: SysIdRoutine.Direction): Command =
            sysIdRoutine.quasistatic(direction)
                .until {
                    if (direction == SysIdRoutine.Direction.kForward) {
                        inputs.position.rotations > 0.4
                    } else {
                        inputs.position.rotations < -0.3
                    }
                }
                .andThen(runOnce { io.driveVoltage(0.0) })

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
internal val FLYWHEEL_PID_GAINS = PIDGains(0.0029805, 0.0, 0.0)
internal val FLYWHEEL_FF_GAINS = MotorFFGains(0.26294, 0.10896 / TAU, 0.010373 / TAU)
