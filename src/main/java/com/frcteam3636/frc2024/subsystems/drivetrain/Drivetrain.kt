package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.frcteam3636.frc2024.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.frc2024.utils.swerve.toCornerSwerveModuleStates
import com.pathplanner.lib.commands.FollowPathHolonomic
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

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
            Pose2d() // initial pose
            // TODO: add odometry standard deviation
        )

    init {
        Pathfinding.setPathfinder(RemoteADStarAK())
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Drivetrain", inputs)

        poseEstimator.update(
            inputs.gyroRotation.toRotation2d(),
            inputs.measuredPositions.toTypedArray()
        )

        Logger.recordOutput("Drivetrain/EstimatedPose", estimatedPose)
    }

    // The rotation of the robot as measured by the gyro.
    var gyroRotation
        get() = inputs.gyroRotation
        set(value) {
            io.resetGyro(value)
        }

    private var moduleStates: PerCorner<SwerveModuleState>
        // Get the measured module states from the inputs.
        get() = inputs.measuredStates
        // Set the desired module states.
        set(value) {
            io.setDesiredStates(value)
            Logger.recordOutput("Drivetrain/DesiredStates", *value.toTypedArray())
        }

    // The current speed of chassis relative to the ground, assuming that the wheels have perfect
    // traction with the ground.
    var chassisSpeeds: ChassisSpeeds
        // Get the chassis speeds from the measured module states.
        get() = kinematics.cornerStatesToChassisSpeeds(inputs.measuredStates)
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
        // set the modules to radiate outwards from the chassis origin
        moduleStates =
            MODULE_POSITIONS.map { position -> SwerveModuleState(0.0, position.rotation) }
    }

    // Get the estimated pose of the drivetrain using the pose estimator.
    val estimatedPose: Pose2d
        get() = poseEstimator.estimatedPosition

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run {
            chassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationJoystick.y,
                    translationJoystick.x,
                    rotationJoystick.x,
                    gyroRotation.toRotation2d()
                )
        }

    fun driveToPose(target: Pose2d): Command {
        Pathfinding.setStartPosition(estimatedPose.translation)
        Pathfinding.setGoalPosition(target.translation)
        val path = Pathfinding.getCurrentPath(DEFAULT_PATHING_CONSTRAINTS, GoalEndState(0.0, target.rotation))

        return FollowPathHolonomic(
            path,
            this::estimatedPose,
            this::chassisSpeeds,
            this::chassisSpeeds::set,
            TRANSLATION_PID_CONSTANTS,
            ROTATION_PID_CONSTANTS,
            MAX_MODULE_SPEED,
            DRIVE_BASE_RADIUS,
            REPLANNING_CONFIG,
            ::isRed,
            this
        )
    }

    private fun isRed(): Boolean {
        return DriverStation.getAlliance().equals(DriverStation.Alliance.Red)
    }
}

abstract class DrivetrainIO {
    abstract val gyro: Gyro
    abstract val modules: PerCorner<out SwerveModule>

    class Inputs : LoggableInputs {
        var gyroRotation: Rotation3d = Rotation3d()
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


//Pathing
internal val DEFAULT_PATHING_CONSTRAINTS = PathConstraints(0.0, 0.0, 0.0, 0.0)
internal val TRANSLATION_PID_CONSTANTS = PIDConstants(0.0, 0.0, 0.0)
internal val ROTATION_PID_CONSTANTS = PIDConstants(0.0, 0.0, 0.0)
internal const val MAX_MODULE_SPEED = 0.0
internal const val DRIVE_BASE_RADIUS = 0.0
internal const val ERROR_REPLANNING_THRESHOLD = 0.0762 //3 inches in meters
internal const val ERROR_SPIKE_REPLANNING_THRESHOLD = ERROR_REPLANNING_THRESHOLD / 2
internal val REPLANNING_CONFIG =
    ReplanningConfig(true, true, ERROR_REPLANNING_THRESHOLD, ERROR_SPIKE_REPLANNING_THRESHOLD)


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
