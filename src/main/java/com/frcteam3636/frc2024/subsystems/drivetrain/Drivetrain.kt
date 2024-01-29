package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.frcteam3636.frc2024.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.frc2024.utils.swerve.toCornerSwerveModuleStates
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

class DrivetrainOdometryThread : Runnable {

    override fun run() {
        while (!Thread.interrupted()) {
            Drivetrain.updateInputs()
            Drivetrain.updateOdometry()
        }
    }
}

data class VisionMeasurement(val stddev: Matrix<N3, N1>, val pose: Pose3d, val timestamp: Double)
// A singleton object representing the drivetrain.
object Drivetrain : Subsystem {
    private val io =
            if (RobotBase.isReal()) {
                DrivetrainIOReal()
            } else {
                DrivetrainIOSim()
            }
    private val inputs = DrivetrainIO.Inputs()

    // Create swerve drivetrain kinematics using the translation parts of the module positions.
    private val kinematics =
            SwerveDriveKinematics(
                    *MODULE_POSITIONS.map(Pose2d::getTranslation).toList().toTypedArray()
            )

    private val poseEstimator =
            SwerveDrivePoseEstimator(
                    kinematics, // swerve drive kinematics
                    inputs.gyroRotation.toRotation2d(), // initial gyro rotation
                    inputs.measuredPositions.toTypedArray(), // initial module positions
                    Pose2d(), // initial pose
                    ODOMETRY_STD_DEV,
                    VecBuilder.fill(0.0, 0.0, 0.0) //will be overwritten be each added vision measurement
            )

    init {
        CommandScheduler.getInstance().registerSubsystem(this)
    }

    fun updateInputs() {
        io.updateInputs(inputs)
    }

    fun updateOdometry() {
        synchronized(this) {
            poseEstimator.update(
                    inputs.gyroRotation.toRotation2d(),
                    inputs.measuredPositions.toTypedArray()
            )
        }
    }

    fun addVisionMeasurement(measurement: VisionMeasurement) {
        synchronized(this) {
            poseEstimator.addVisionMeasurement(
                    measurement.pose.toPose2d(),
                    measurement.timestamp,
                    measurement.stddev
            )
        }
    }

    override fun periodic() {
        Logger.processInputs("Drivetrain", inputs)
        Logger.recordOutput("Drivetrain/EstimatedPose", estimatedPose)
    }

    // The rotation of the robot as measured by the gyro.
    var gyroRotation
        get(): Rotation3d = inputs.gyroRotation
        set(value) = io.resetGyro(value)

    private var moduleStates: PerCorner<SwerveModuleState>
        // Get the measured module states from the inputs.
        get() {
            synchronized(this) {
                return inputs.measuredStates
            }
        }
        // Set the desired module states.
        set(value) {
            synchronized(this) {
                io.setDesiredStates(value)
                Logger.recordOutput("Drivetrain/DesiredStates", *value.toTypedArray())
            }
        }

    // The current speed of chassis relative to the ground, assuming that the wheels have perfect
    // traction with the ground.
    var chassisSpeeds: ChassisSpeeds
        // Get the chassis speeds from the measured module states.
        get(): ChassisSpeeds = kinematics.cornerStatesToChassisSpeeds(inputs.measuredStates)
        // Set the drivetrain to move with the given chassis speeds.
        //
        // Note that the speeds are relative to the chassis, not the field.
        set(value) {

            val states = kinematics.toCornerSwerveModuleStates(value)
            // TODO: desaturate states
            moduleStates = states
        }

    // Set the drivetrain to an X-formation to passively prevent movement in any direction.
    fun brake() {

        moduleStates =
                MODULE_POSITIONS.map { position -> SwerveModuleState(0.0, position.rotation) }

        // set the modules to radiate outwards from the chassis origin
    }

    // Get the estimated pose of the drivetrain using the pose estimator.
    val estimatedPose: Pose2d
        get() = poseEstimator.estimatedPosition

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
            synchronized(this) {
                run {
                    chassisSpeeds =
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translationJoystick.y,
                                    translationJoystick.x,
                                    rotationJoystick.x,
                                    gyroRotation.toRotation2d()
                            )
                }
            }
}

abstract class DrivetrainIO {
    abstract val gyro: Gyro
    abstract val modules: PerCorner<out SwerveModule>

    class Inputs : LoggableInputs {
        var gyroRotation: Rotation3d = Rotation3d()
            set(value: Rotation3d) {
                synchronized(this) { field = value }
            }
        var measuredStates: PerCorner<SwerveModuleState> =
                PerCorner.generate { SwerveModuleState() }
        var measuredPositions: PerCorner<SwerveModulePosition> =
                PerCorner.generate { SwerveModulePosition() }

        override fun fromLog(table: LogTable?) {
            gyroRotation = table?.get("GyroRotation", gyroRotation)!![0]
            measuredStates =
                    PerCorner.fromConventionalArray(
                            table.get("MeasuredStates", *measuredStates.toTypedArray())
                    )
            measuredPositions =
                    PerCorner.fromConventionalArray(
                            table.get("MeasuredPositions", *measuredPositions.toTypedArray())
                    )
        }

        override fun toLog(table: LogTable?) {
            table?.put("GyroRotation", gyroRotation)
            table?.put("MeasuredStates", *measuredStates.toTypedArray())
            table?.put("MeasuredPositions", *measuredPositions.toTypedArray())
        }
    }

    fun updateInputs(inputs: Inputs) {
        gyro.periodic()
        modules.forEach(SwerveModule::periodic)

        inputs.gyroRotation = gyro.rotation
        inputs.measuredStates = modules.map { it.state }
        inputs.measuredPositions = modules.map { it.position }
    }

    fun resetGyro(rotation: Rotation3d) {
        gyro.rotation = rotation
    }

    fun setDesiredStates(states: PerCorner<SwerveModuleState>) {
        modules.zip(states).forEach { (module, state) -> module.desiredState = state }
    }
}

class DrivetrainIOReal : DrivetrainIO() {
    override val gyro = GyroNavX()
    override val modules =
            MODULE_CAN_IDS.zip(MODULE_POSITIONS).map { (can, pose) ->
                val (driving, turning) = can
                MAXSwerveModule(driving, turning, pose.rotation)
            }
}

class DrivetrainIOSim : DrivetrainIO() {
    override val modules = PerCorner.generate { SimSwerveModule() }
    override val gyro = GyroSim(modules.map { it })
}

// Constants
internal val WHEEL_BASE: Double = Units.inchesToMeters(13.0)
internal val TRACK_WIDTH: Double = Units.inchesToMeters(14.0)

internal val ODOMETRY_STD_DEV: Matrix<N3, N1> = VecBuilder.fill(0.0, 0.0, 0.0)

internal val MODULE_POSITIONS =
        PerCorner(
                frontLeft =
                        Pose2d(
                                Translation2d(WHEEL_BASE, TRACK_WIDTH) / 2.0,
                                Rotation2d.fromDegrees(-90.0)
                        ),
                frontRight =
                        Pose2d(
                                Translation2d(WHEEL_BASE, -TRACK_WIDTH) / 2.0,
                                Rotation2d.fromDegrees(180.0)
                        ),
                backRight =
                        Pose2d(
                                Translation2d(-WHEEL_BASE, TRACK_WIDTH) / 2.0,
                                Rotation2d.fromDegrees(0.0)
                        ),
                backLeft =
                        Pose2d(
                                Translation2d(-WHEEL_BASE, -TRACK_WIDTH) / 2.0,
                                Rotation2d.fromDegrees(90.0)
                        )
        )

internal val MODULE_CAN_IDS =
        PerCorner(
                frontLeft =
                        Pair(
                                CTREMotorControllerId.FrontLeftDrivingMotor,
                                REVMotorControllerId.FrontLeftTurningMotor
                        ),
                frontRight =
                        Pair(
                                CTREMotorControllerId.FrontRightDrivingMotor,
                                REVMotorControllerId.FrontRightTurningMotor
                        ),
                backRight =
                        Pair(
                                CTREMotorControllerId.BackRightDrivingMotor,
                                REVMotorControllerId.BackRightTurningMotor
                        ),
                backLeft =
                        Pair(
                                CTREMotorControllerId.BackLeftDrivingMotor,
                                REVMotorControllerId.BackLeftTurningMotor
                        ),
        )
