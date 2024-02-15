package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.math.PIDController
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.frcteam3636.frc2024.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.frc2024.utils.swerve.toCornerSwerveModuleStates
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.simulation.VisionSystemSim

// A singleton object representing the drivetrain.
object Drivetrain : Subsystem {
    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> DrivetrainIOSim()
        Robot.Model.COMPETITION -> DrivetrainIOReal(MODULE_POSITIONS.zip(MODULE_CAN_IDS_COMP).map { (position, ids) ->
            val (driveId, turnId) = ids
            MAXSwerveModule(
                DrivingTalon(driveId),
                turnId,
                position.rotation
            )
        })

        Robot.Model.PRACTICE -> DrivetrainIOReal(MODULE_POSITIONS.zip(MODULE_CAN_IDS_PRACTICE).map { (position, ids) ->
            val (driveId, turnId) = ids
            MAXSwerveModule(
                DrivingSparkMAX(driveId),
                turnId,
                position.rotation
            )
        })
    }
    private val inputs = DrivetrainIO.Inputs()


    private val photonVisionSimSystem = when (Robot.model) {
        Robot.Model.SIMULATION -> VisionSystemSim("main").apply {
            addAprilTags(APRIL_TAG_FIELD_LAYOUT)
        }

        else -> null
    }

    private val absolutePoseIOs = when (Robot.model) {
        Robot.Model.SIMULATION -> mapOf(
            "Fljorg" to PhotonVisionPoseIOSim(
                "fljorg",
                CHASSIS_TO_CAMERA_TRANSFORMS["fljorg"]!!,
                photonVisionSimSystem!!
            ),
            "Bloop" to PhotonVisionPoseIOSim(
                "bloop",
                CHASSIS_TO_CAMERA_TRANSFORMS["bloop"]!!,
                photonVisionSimSystem
            ),
            "Freedom" to PhotonVisionPoseIOSim(
                "freedom",
                CHASSIS_TO_CAMERA_TRANSFORMS["freedom"]!!,
                photonVisionSimSystem
            ),
            "Brack" to PhotonVisionPoseIOSim(
                "brack",
                CHASSIS_TO_CAMERA_TRANSFORMS["brack"]!!,
                photonVisionSimSystem
            )
        )
        Robot.Model.COMPETITION -> emptyMap()
        Robot.Model.PRACTICE -> mapOf(
            "Fljorg" to PhotonVisionPoseIOReal(
                "fljorg",
                CHASSIS_TO_CAMERA_TRANSFORMS["fljorg"]!!
            ),
            "Bloop" to PhotonVisionPoseIOReal(
                "bloop",
                CHASSIS_TO_CAMERA_TRANSFORMS["bloop"]!!
            ),
            "Freedom" to PhotonVisionPoseIOReal(
                "freedom",
                CHASSIS_TO_CAMERA_TRANSFORMS["freedom"]!!
            ),
            "Brack" to PhotonVisionPoseIOReal(
                "brack",
                CHASSIS_TO_CAMERA_TRANSFORMS["brack"]!!
            )
        )
    }.mapValues { Pair(it.value, AbsolutePoseIO.Inputs()) }

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
            WHEEL_ODOMETRY_STD_DEV,
            VecBuilder.fill(0.0, 0.0, 0.0) //will be overwritten be each added vision measurement
        )

    // Used as the "true" pose estimator in simulation
    private val simualatedPoseOdometry = when (Robot.model) {
        Robot.Model.SIMULATION -> SwerveDriveOdometry(
            SwerveDriveKinematics(
                *MODULE_POSITIONS.map(Pose2d::getTranslation).toList().toTypedArray()
            ),
            inputs.gyroRotation.toRotation2d(),
            inputs.measuredPositions.toTypedArray()
        )

        else -> null
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Drivetrain", inputs)

        if (Robot.model == Robot.Model.SIMULATION) {
            simualatedPoseOdometry!!.update(
                inputs.gyroRotation.toRotation2d(),
                inputs.measuredPositions.toTypedArray()
            )

            photonVisionSimSystem!!.update(simualatedPoseOdometry.poseMeters)

            Logger.recordOutput("Drivetrain/Simulated Pose", simualatedPoseOdometry.poseMeters)
        }

        absolutePoseIOs.forEach { (name, ioPair) ->
            val (io, inputs) = ioPair

            io.updateInputs(inputs)
            Logger.processInputs("Drivetrain/$name", ioPair.second)

            inputs.measurement?.let { poseEstimator.addAbsolutePoseMeasurement(it) }
        }

        poseEstimator.update(
            inputs.gyroRotation.toRotation2d(),
            inputs.measuredPositions.toTypedArray()
        )
        Logger.recordOutput("Drivetrain/Estimated Pose", estimatedPose)
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
            Logger.recordOutput("Drivetrain/Desired States", *value.toTypedArray())
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

    fun driveWithController(controller: CommandXboxController): Command =
        run {
            chassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    controller.leftX,
                    controller.leftY,
                    controller.rightX,
                    gyroRotation.toRotation2d()
                )
        }

    fun driveWithJoystickPointingTowards(translationJoystick: Joystick, target: Translation2d): Command =
        run {
            val magnitude = ROTATION_PID_CONTROLLER.calculate(
                estimatedPose.rotation.radians,
                target.minus(estimatedPose.translation).angle.radians
            )

            val chassisSpeeds = ChassisSpeeds(
                translationJoystick.x * FREE_SPEED,
                translationJoystick.y * FREE_SPEED,
                0.0
            )
        }

    fun zeroGyro() {
        gyroRotation = Rotation3d()
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
            gyroRotation = table?.get("Gyro Rotation", gyroRotation)!![0]
            measuredStates =
                PerCorner.fromConventionalArray(
                    table.get("Measured States", *measuredStates.toTypedArray())
                )
            measuredPositions =
                PerCorner.fromConventionalArray(
                    table.get("Measured Positions", *measuredPositions.toTypedArray())
                )
        }

        override fun toLog(table: LogTable?) {
            table?.put("Gyro Rotation", gyroRotation)
            table?.put("Measured States", *measuredStates.toTypedArray())
            table?.put("Measured Positions", *measuredPositions.toTypedArray())
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

class DrivetrainIOReal(override val modules: PerCorner<out SwerveModule>) : DrivetrainIO() {
    override val gyro = GyroNavX()
}

class DrivetrainIOSim : DrivetrainIO() {
    override val modules = PerCorner.generate { SimSwerveModule() }
    override val gyro = GyroSim(modules.map { it })
}

// Physical dimensions
internal val WHEEL_BASE: Measure<Distance> = Inches.of(13.0)
internal val TRACK_WIDTH: Measure<Distance> = Inches.of(14.0)
internal val MODULE_POSITIONS =
    PerCorner(
        frontLeft =
        Pose2d(
            Translation2d(WHEEL_BASE.baseUnitMagnitude(), TRACK_WIDTH.baseUnitMagnitude()) / 2.0,
            Rotation2d.fromDegrees(-90.0)
        ),
        backLeft =
        Pose2d(
            Translation2d(WHEEL_BASE.negate().baseUnitMagnitude(), TRACK_WIDTH.baseUnitMagnitude()) / 2.0,
            Rotation2d.fromDegrees(0.0)
        ),
        backRight =
        Pose2d(
            Translation2d(WHEEL_BASE.negate().baseUnitMagnitude(), TRACK_WIDTH.negate().baseUnitMagnitude()) / 2.0,
            Rotation2d.fromDegrees(90.0)
        ),
        frontRight =
        Pose2d(
            Translation2d(WHEEL_BASE.baseUnitMagnitude(), TRACK_WIDTH.negate().baseUnitMagnitude()) / 2.0,
            Rotation2d.fromDegrees(180.0)
        ),
    )
internal val CHASSIS_TO_CAMERA_TRANSFORMS = mapOf(
    "fljorg" to Transform3d(Translation3d(0.1175, 0.3175, 0.0), Rotation3d(0.0, 1.31, 0.785)),
    "bloop" to Transform3d(Translation3d(-0.1175, 0.3175, 0.0), Rotation3d(0.0, 1.31, 1.570)),
    "freedom" to Transform3d(Translation3d(0.1175, -0.3175, 0.0), Rotation3d(0.0, 1.31, 0.0)),
    "brack" to Transform3d(Translation3d(-0.1175, -0.3175, 0.0), Rotation3d(0.0, 1.31, 4.71))
)

// Performance characteristics
internal val FREE_SPEED = 15.0

// Wheel odometry
internal val WHEEL_ODOMETRY_STD_DEV = VecBuilder.fill(0.2, 0.2, 0.005)

// Controller gains
internal val ROTATION_PID_CONTROLLER = PIDController(PIDGains(0.3, 0.0, 0.0))

// CAN IDs
internal val MODULE_CAN_IDS_COMP =
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

internal val MODULE_CAN_IDS_PRACTICE =
    PerCorner(
        frontLeft =
        Pair(
            REVMotorControllerId.FrontLeftDrivingMotor,
            REVMotorControllerId.FrontLeftTurningMotor
        ),
        frontRight =
        Pair(
            REVMotorControllerId.FrontRightDrivingMotor,
            REVMotorControllerId.FrontRightTurningMotor
        ),
        backRight =
        Pair(
            REVMotorControllerId.BackRightDrivingMotor,
            REVMotorControllerId.BackRightTurningMotor
        ),
        backLeft =
        Pair(
            REVMotorControllerId.BackLeftDrivingMotor,
            REVMotorControllerId.BackLeftTurningMotor
        ),
    )

enum class OrientationTarget(val position: Translation2d) {
    Speaker(Translation2d()),
    Amp(Translation2d()),
    Source(Translation2d())
}
