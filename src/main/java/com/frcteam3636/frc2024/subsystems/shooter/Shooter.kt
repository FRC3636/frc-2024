package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.BLACK
import com.frcteam3636.frc2024.BLUE
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.WHITE
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.utils.math.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan
import kotlin.math.pow

object Shooter {

    fun shootCommand(velocity: Double, spin: Double): Command = ParallelCommandGroup(
        Flywheels.rev(velocity, spin),
        Feeder.feedCommand().withTimeout(0.1)
    )

    object Flywheels : Subsystem {
        private val io: FlywheelIO = when (Robot.model) {
            Robot.Model.SIMULATION -> FlywheelIOSim()
            Robot.Model.COMPETITION, Robot.Model.PRACTICE -> FlywheelIOReal()
        }
        private val inputs = FlywheelIO.Inputs()

        private var setpointLeft = RadiansPerSecond.zero()
        private var setpointRight = RadiansPerSecond.zero()

        var lastVelocity = RadiansPerSecond.zero()


        private val pidControllerLeft = PIDController(FLYWHEEL_PID_GAINS)
        private val pidControllerRight = PIDController(FLYWHEEL_PID_GAINS)
        private val ffController = SimpleMotorFeedforward(FLYWHEEL_FF_GAINS)

        val acceleration: Measure<Velocity<Velocity<Angle>>>
            get() {
                return inputs.leftSpeed.minus(lastVelocity).per(Second.of(Robot.period))
            }


        val aboveIntakeThreshold: Boolean
            get() {
                return Amps.of(inputs.leftCurrent.baseUnitMagnitude().pow(3)) > FLYWHEEL_INTAKE_CURRENT_THRESHOLD
            }


        val atDesiredVelocity: Boolean
            get() {
                val velocityDifference = (inputs.leftSpeed.minus(setpointLeft).baseUnitMagnitude().absoluteValue)
                Logger.recordOutput("Shooter/Flywheels/velocity difference", velocityDifference)
                Logger.recordOutput(
                    "shooter/Flywheels/at deired velocirty", velocityDifference <
                            FLYWHEEL_VELOCITY_TOLERANCE.baseUnitMagnitude()
                )
                return velocityDifference <
                        FLYWHEEL_VELOCITY_TOLERANCE.baseUnitMagnitude() && inputs.leftSpeed > RadiansPerSecond.of(30.0)
            }

        override fun periodic() {
            io.updateInputs(inputs)

            Logger.processInputs("Shooter/Flywheels", inputs)

            flywheelLigament.color = if (inputs.leftSpeed > RotationsPerSecond.of(1.0)) {
                BLUE
            } else {
                WHITE
            }

            Logger.recordOutput("Shooter/Flywheels/at desired velocity", atDesiredVelocity)
            Logger.recordOutput("Shooter", mechanism)
            Logger.recordOutput("Shooter/Flywheels/above current threshold", aboveIntakeThreshold)
            Logger.recordOutput(
                "Shooter/Flywheels/average current squared",
                inputs.leftCurrent.baseUnitMagnitude().pow(3)
            )

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

        }


        /** Shoot a ball at a given velocity and spin (in rad/s). */
        fun rev(velocity: Double, spin: Double): Command =
            // start the flywheels
            runEnd({
                val tangentialVelocity = spin * FLYWHEEL_SIDE_SEPERATION / 2.0

                setpointLeft = RadiansPerSecond.of((velocity - tangentialVelocity))
                setpointRight = RadiansPerSecond.of((velocity + tangentialVelocity))
            }, {
                setpointLeft = RadiansPerSecond.zero()
                setpointRight = RadiansPerSecond.zero()
            })

        fun outtake(): Command =
            runEnd({
                setpointLeft = RadiansPerSecond.of(30.0)
                setpointRight = RadiansPerSecond.of(30.0)
            }, {
                setpointLeft = RadiansPerSecond.zero()
                setpointRight = RadiansPerSecond.zero()
            })

        fun intake(): Command {
            return runEnd({
                setpointLeft = RadiansPerSecond.of(-50.0)
                setpointRight = RadiansPerSecond.of(-50.0)
            }, {
                setpointLeft = RadiansPerSecond.zero()
                setpointRight = RadiansPerSecond.zero()
            })
        }

        fun getState(log: SysIdRoutineLog) {
            log.motor("left-flywheels")
                .voltage(
                    Volts.of(inputs.leftVoltage)
                )
                .angularPosition(inputs.leftPos)
                .angularVelocity(
                    inputs.leftSpeed
                )
            log.motor("right-flywheels")
                .voltage(
                    Volts.of(inputs.rightVoltage)
                )
                .angularPosition(inputs.rightPos)
                .angularVelocity(
                    inputs.rightSpeed
                )
        }
    }

    object Feeder : Subsystem {
        private val io = when (Robot.model) {
            Robot.Model.SIMULATION -> FeederIOSim()
            else -> FeederIOReal()
        }
        private val inputs = FeederIO.Inputs()


        fun intakeCommand(): Command = Commands.runEnd({
            io.setIndexerVoltage(Volts.of(10.0))
        }, {
            io.setIndexerVoltage(Volts.zero())
        })

        fun pulseCommand(): Command = Commands.runEnd({
            io.setIndexerVoltage(Volts.of(0.3))
        }, {
            io.setIndexerVoltage(Volts.zero())
        }, this)

        fun outtakeCommand(): Command = Commands.runEnd({
            io.setIndexerVoltage(Volts.of(-4.0))
        }, {
            io.setIndexerVoltage(Volts.zero())
        })

        fun feedCommand(): Command = Commands.runEnd({
            io.setIndexerVoltage(Volts.of(-10.0))
        }, {
            io.setIndexerVoltage(Volts.zero())
        })
    }

    object Pivot : Subsystem {
        private val io: PivotIO = when (Robot.model) {
            Robot.Model.SIMULATION -> PivotIOSim()
            Robot.Model.COMPETITION -> PivotIOKraken()
            Robot.Model.PRACTICE -> TODO()
        }
        private val inputs = PivotIO.Inputs()

        var target: Target = Target.PODIUM

//
//        init {
//            io.updateInputs(inputs)
//            zeroPivot()
//        }


        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Pivot", inputs)

            armLigament.angle = inputs.leftPosition.degrees

            Logger.recordOutput("Shooter/IsStowed", isStowed())
            Logger.recordOutput("Shooter", mechanism)
        }

        fun pivotAndStop(goal: Rotation2d): Command = Commands.sequence(runOnce {
            Logger.recordOutput("Shooter/DesiredPosition", goal)
            io.pivotToAndStop(goal)
        }, Commands.waitUntil {
            (abs((goal - inputs.leftPosition).radians) < PIVOT_POSITION_TOLERANCE.radians)
                    && (abs(inputs.leftVelocity.radians) < PIVOT_VELOCITY_TOLERANCE.radians)
        })

        fun isPointingTowards(target: Rotation2d) = Trigger {
            abs((target - inputs.leftPosition).radians) < PIVOT_POSITION_TOLERANCE.radians
        }

        val readyToShoot = Trigger {
            inputs.leftPosition.degrees >= 90
        }

        fun isStowed(): Boolean {

            return (abs((Rotation2d.fromDegrees(-27.0) - inputs.leftPosition).radians) < Rotation2d.fromDegrees(2.5).radians)
        }


        fun setTarget(target: Target): Command {
            return runOnce {
                this.target = target
            }
        }

        fun zeroPivot() {
            io.resetPivotToAbsoluteEncoder()
        }

        fun setVoltage(volts: Measure<Voltage>) {
            io.driveVoltage(volts.magnitude())
        }

        val sysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(Volts.of(1.0).per(Seconds.of(2.0)), Volts.of(1.0), null, null),
            SysIdRoutine.Mechanism(
                {
                    io.driveVoltage(it.baseUnitMagnitude())
                },
                {
                    it.motor("pivot-left")
                        .voltage(Volts.of(inputs.voltageLeft))
                        .angularPosition(Radians.of(inputs.leftPosition.radians))
                        .angularVelocity(RadiansPerSecond.of(inputs.leftVelocity.radians))
                    it.motor("pivot-right")
                        .voltage(Volts.of(inputs.voltageRight))
                        .angularPosition(Radians.of(inputs.rightPosition.radians))
                        .angularVelocity(RadiansPerSecond.of(inputs.rightVelocity.radians))
                },
                this
            )
        )

        fun doDynamicSysId(direction: SysIdRoutine.Direction): Command =
            sysIdRoutine.dynamic(direction)
                .andThen(runOnce { io.driveVoltage(0.0) })

        fun doQuasistaticSysId(direction: SysIdRoutine.Direction): Command =
            sysIdRoutine.quasistatic(direction)
                .andThen(runOnce { io.driveVoltage(0.0) })

        fun followMotionProfile(targetOverride: Target?): Command {
            return run {
                val target = targetOverride ?: this.target
                val position = target.profile.position()
                val velocity = target.profile.velocity()
                Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)
                Logger.recordOutput("Shooter/Pivot/Velocity Setpoint", velocity)
                io.pivotToAndMove(position, velocity)
            }
        }

        fun setBrakeMode(brake: Boolean): Command =
            InstantCommand({
                io.setBrakeMode(brake)
            })


        fun neutralMode(): Command = startEnd({
            io.driveVoltage(0.0)
            io.setBrakeMode(false)
        }, {
            io.setBrakeMode(true)
        })
            .ignoringDisable(true)

        enum class Target(val profile: PivotProfile) {
            AIM(
                PivotProfile(
                    {

                        val speakerPose = Pose2d(
                            Translation2d(SPEAKER_POSE.x, SPEAKER_POSE.y),
                            Rotation2d()
                        )
                        Logger.recordOutput("Drivetrain/Speaker Vector", speakerPose)
                        val distance = speakerPose.translation.minus(Drivetrain.estimatedPose.translation)
                            .minus(Translation2d(0.3, 0.0)).norm

                        Logger.recordOutput("Drivetrain/distance to speaker", distance)
                        val targetHeight = SPEAKER_POSE.z
                        Logger.recordOutput("Drivetrain/Speaker height", targetHeight)
                        Rotation2d((TAU / 2) - atan(targetHeight / distance))
                    },
                    { Rotation2d() }
                )
            ),
            CurrentPosition(
                PivotProfile({ inputs.leftPosition }, { Rotation2d() })
            ),
            SPEAKER(
                PivotProfile(
                    {
                        Rotation2d.fromDegrees(150.0)

                    },
                    {
                        Rotation2d()
                    }
                )
            ),
            AMP(
                PivotProfile(
                    { Rotation2d.fromDegrees(101.0) },
                    { Rotation2d() }
                )
            ),
            PODIUM(
                PivotProfile(
                    { Rotation2d.fromDegrees(10.0) },
                    { Rotation2d() }
                )
            ),
            STOWED(
                PivotProfile(
                    { Rotation2d.fromDegrees(-27.0) },
                    { Rotation2d() }
                )
            )
        }

    }

    object Amp : Subsystem {
        val io = AmpMechIOReal()
        val inputs = AmpMechIO.Inputs()

        var posReference: Rotation2d = Rotation2d(0.0)

        fun pivotTo(pos: Rotation2d): Command = runOnce {
            Logger.recordOutput("Shooter/Amp/setpoint", pos)
            posReference = pos
        }

        fun stow(): Command {
            return Commands.sequence(
                runOnce {
                    io.setVoltage(Volts.of(-2.0))
                },
                WaitCommand(0.3),
                runOnce {
                    io.zero()
                }
            ).finallyDo(
                Runnable {
                    io.setVoltage(Volts.of(0.0))
                }
            )
        }

        override fun periodic() {
            io.updateInputs(inputs)
            io.pivotTo(posReference)
            Logger.processInputs("Shooter/AmpMech", inputs)
        }
    }

    // Register the two subsystems which together form the shooter.
    fun register() {
        Flywheels.register()
        Pivot.register()
        Amp.register()
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

data class PivotProfile(
    val position: () -> Rotation2d,
    val velocity: () -> Rotation2d
)


//amps
internal val FLYWHEEL_INTAKE_CURRENT_THRESHOLD = Amps.of(30000.0)
val SPEAKER_POSE = when (DriverStation.getAlliance().getOrNull()) {
    DriverStation.Alliance.Red -> Translation3d(16.511, 8.21055 - 2.6, Units.inchesToMeters(84.5))
    else -> Translation3d(0.0, 8.21055 - 2.6, Units.inchesToMeters(84.5))
}

internal val PIVOT_POSITION_TOLERANCE = Rotation2d.fromDegrees(2.0)
internal val PIVOT_VELOCITY_TOLERANCE = Rotation2d.fromDegrees(2.0)
internal val AMP_MECH_POSITION_TOLERANCE = Rotation2d.fromDegrees(3.0)
internal val FLYWHEEL_VELOCITY_TOLERANCE = RadiansPerSecond.of(13.0)

internal val FLYWHEEL_RADIUS = Units.inchesToMeters(1.5)
internal val FLYWHEEL_SIDE_SEPERATION = Units.inchesToMeters(9.0)
internal val FLYWHEEL_PID_GAINS = PIDGains(0.0029805, 0.0, 0.0)
internal val FLYWHEEL_FF_GAINS = MotorFFGains(0.26294, 0.10896 / TAU, 0.010373 / TAU)