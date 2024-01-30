package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.frcteam3636.frc2024.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.frc2024.utils.swerve.toCornerSwerveModuleStates
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
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
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import java.util.*


class DrivetrainOdometryThread : Runnable {

    override fun run() {
        while (!Thread.interrupted()) {
            Drivetrain.updateInputs()
            Drivetrain.updateOdometry()
        }
    }
}

data class VisionMeasurement(val stddev: Matrix<N3, N1>, val pose: Pose3d, val timestamp: Double)
interface VisionBackend {
    fun getMeasurement(): Optional<VisionMeasurement>
}

class PhotonAprilTagCamera(name: String, cameraTransform: Transform3d) : VisionBackend {
    private val io =
        if (RobotBase.isReal()) {
            AprilTagPhotonVisionIOReal(name, cameraTransform)
        } else {
            throw NotImplementedError()
        }

    override fun getMeasurement(): Optional<VisionMeasurement> {
        return io.result.flatMap { result ->
            {
                if (result.targetsUsed.first().bestCameraToTarget.translation.norm > APRIL_TAG_DISTANCE_FILTER
                    || result.targetsUsed.first().poseAmbiguity > APRIL_TAG_DISTANCE_FILTER
                ) {
                    Optional.empty<VisionMeasurement>()
                }

                if (result.estimatedPose.x < 0 || result.estimatedPose.x > FIELD_LENGTH ||
                    result.estimatedPose.y < 0 || result.estimatedPose.y > FIELD_WIDTH
                ) {

                    Optional.empty<VisionMeasurement>()
                }


                Optional.of(
                    VisionMeasurement(
                        APRIL_TAG_STD_DEV(result.targetsUsed.first().bestCameraToTarget.x, result.targetsUsed.size),
                        result.estimatedPose,
                        result.timestampSeconds
                    )
                )
            }()
        }
    }
}

class AprilTagPhotonVisionIOReal(private val name: String, private val cameraTransform: Transform3d) :
    AprilTagPhotonVisionIO() {
    override val camera = PhotonCamera(name).apply {
        driverMode = false
        pipelineIndex = 0
    }

    val fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)

    override val poseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
        fieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        cameraTransform
    )

    override fun updateInputs(inputs: Inputs) {
        inputs.targetCorners = result.get().targetsUsed
            .flatMap { target ->
                target.detectedCorners
            }.map { corner ->
                Translation2d(corner.x, corner.y)
            }

        inputs.targetIDs = result.get().targetsUsed
            .map { target ->
                target.fiducialId
            }
    }

}

abstract class AprilTagPhotonVisionIO {

    abstract val camera: PhotonCamera
    abstract val poseEstimator: PhotonPoseEstimator
    val result: Optional<EstimatedRobotPose>
        get() = poseEstimator.update()

    class Inputs : LoggableInputs {
        var targetCorners: List<Translation2d> = mutableListOf() //unordered
        var targetIDs: List<Int> = mutableListOf()

        override fun toLog(table: LogTable?) {
            table?.put("Target Corners Used", *targetCorners.toTypedArray())
            table?.put("Target IDs Used", targetIDs.toIntArray())
        }

        override fun fromLog(table: LogTable?) {
            targetCorners = table?.get("Target Corners Used", *targetCorners.toTypedArray())!!.toList()
            targetIDs = table?.get("Target IDs Used", targetIDs.toIntArray())!!.toList()
        }
    }

    abstract fun updateInputs(inputs: Inputs)
}

// A singleton object representing the drivetrain.
object Drivetrain : Subsystem {
    private val io =
        if (RobotBase.isReal()) {
            DrivetrainIOReal()
        } else {
            DrivetrainIOSim()
        }
    private val inputs = DrivetrainIO.Inputs()

    private val cameras = PHOTON_CAMERAS.map { (name, transform) -> PhotonAprilTagCamera(name, transform) }

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

    fun addVisionMeasurement(measurement: Optional<VisionMeasurement>) {
        synchronized(this) {
            if (measurement.isPresent) {
                poseEstimator.addVisionMeasurement(
                    measurement.get().pose.toPose2d(),
                    measurement.get().timestamp,
                    measurement.get().stddev
                )
            }
        }
    }

    override fun periodic() {
        Logger.processInputs("Drivetrain", inputs)
        Logger.recordOutput("Drivetrain/EstimatedPose", estimatedPose)
        for (camera in cameras) {
            addVisionMeasurement(camera.getMeasurement())
        }
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
internal const val APRIL_TAG_DISTANCE_FILTER = 0.0
internal const val APRIL_TAG_AMBIGUITY_FILTER = 0.0
internal val APRIL_TAG_STD_DEV = { distance: Double, count: Int ->
    val distanceMultiplier = Math.pow(distance - ((count - 1) * 2), 2.0)
    val translationalStdDev = (0.05 / (count)) * distanceMultiplier + 0.0
    val rotationalStdDev = 0.2 * distanceMultiplier + 0.1
    VecBuilder.fill(
        translationalStdDev,
        translationalStdDev,
        rotationalStdDev
    )
}


//field constants
internal const val FIELD_LENGTH = 0.0
internal const val FIELD_WIDTH = 0.0

internal val PHOTON_CAMERAS: List<Pair<String, Transform3d>> = listOf()

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
