package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.TalonFXStatusProvider
import com.frcteam3636.frc2024.utils.math.PIDController
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.frcteam3636.frc2024.utils.math.TAU
import com.frcteam3636.frc2024.utils.math.toPPLib
import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.frcteam3636.frc2024.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.frc2024.utils.swerve.toCornerSwerveModuleStates
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.VecBuilder
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
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import java.util.*
import kotlin.math.abs

// A singleton object representing the drivetrain.
object Drivetrain : Subsystem, TalonFXStatusProvider {
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


    private val absolutePoseIOs = mapOf(
//        "Fljorg" to PhotonVisionPoseIOReal(
//            "fljorg",
//            Transform3d(
//                Translation3d(0.1175, 0.3175, 0.0),
//                Rotation3d(0.0, 0.0, PI * 0.25) + Rotation3d(0.0, 1.31, 0.0)
//            )
//        ),
//        "Bloop" to PhotonVisionPoseIOReal(
//            "bloop",
//            Transform3d(
//                Translation3d(-0.1175, 0.3175, 0.45),
//                Rotation3d(0.0, 0.0, PI * 0.5) + Rotation3d(0.0, 1.31, 0.0)
//            )
//        ),
//        "Freedom" to PhotonVisionPoseIOReal(
//            "freedom",
//            Transform3d(
//                Translation3d(0.1175, -0.3175, 0.0),
//                Rotation3d(0.0, 0.0, PI + (PI * 0.75)) + Rotation3d(0.0, 1.31, 0.0)
//            )
//        ),
//        "Brack" to PhotonVisionPoseIOReal(
//            "brack",
//            Transform3d(
//                Translation3d(-0.1175, -0.3175, 0.0),
//                Rotation3d(0.0, 0.0, PI + (PI * 0.5)) + Rotation3d(0.0, 1.31, 0.0)
//            )
//        ),
//        "Blowfish" to PhotonVisionPoseIOReal(
//            "blowfish",
//            Transform3d(
//                Translation3d(-0.3656, -0.2794, 0.22),
//                Rotation3d(0.0, 0.0, PI + (PI * 0.5)) + Rotation3d(0.0, 1.31, 0.0)
//            )
//        ),
        "Limelight" to LimelightPoseIOReal(
            "limelight",
        )
    ).mapValues { Pair(it.value, AbsolutePoseIO.Inputs()) }

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
            VecBuilder.fill(0.7, 0.7, 999999.0) //will be overwritten be each added vision measurement
        )

    val gyroConnected
        get() = io.gyro.connected

    val allCamerasConnected
        get() = absolutePoseIOs.values.all { it.first.cameraConnected }

    init {

        Pathfinding.setPathfinder(
            when (Robot.model) {
                Robot.Model.SIMULATION -> LocalADStarAK()
                Robot.Model.COMPETITION, Robot.Model.PRACTICE -> RemoteADStarAK()
            }
        )

        AutoBuilder.configureHolonomic(
            this::estimatedPose,
            this::estimatedPose::set,
            this::chassisSpeeds,
            this::chassisSpeeds::set,
            PATH_FOLLOWER_CONFIG,
            { DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red) },
            this
        )

    }

    override fun periodic() {
        val entries = DoubleArray(6)
        entries[0] = Units.radiansToDegrees(io.gyro.rotation.z)
        entries[1] = 0.0
        entries[2] = 0.0
        entries[3] = 0.0
        entries[4] = 0.0
        entries[5] = 0.0
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("robot_orientation_set").setDoubleArray(entries)

        io.updateInputs(inputs)
        Logger.processInputs("Drivetrain", inputs)

        absolutePoseIOs.forEach { (name, ioPair) ->
            val (io, inputs) = ioPair

            io.updateInputs(inputs)
            Logger.processInputs("Absolute Pose/$name", inputs)
            Logger.recordOutput("Absolute Pose/$name/Is Null", inputs.measurement == null)

            inputs.measurement?.let {
                poseEstimator.addAbsolutePoseMeasurement(it)
                Logger.recordOutput("Drivetrain/Last Added Pose", it.pose)
            }
        }

        poseEstimator.update(
            inputs.gyroRotation.toRotation2d(),
            inputs.measuredPositions.toTypedArray()
        )



        Logger.recordOutput("Drivetrain/Gyro Rotation", inputs.gyroRotation.toRotation2d())
        Logger.recordOutput("Drivetrain/Estimated Pose", estimatedPose)
    }

    // The rotation of the robot as measured by the gyro.
    var gyroRotation
        get() = inputs.gyroRotation
        set(value) {
            io.resetGyro(value)
        }

    val gyroRate
        get() = io.gyro.rate

    private var moduleStates: PerCorner<SwerveModuleState>
        // Get the measured module states from the inputs.
        get() = inputs.measuredStates
        // Set the desired module states.
        set(value) {
            synchronized(this) {
                val stateArr = value.toTypedArray()
                SwerveDriveKinematics.desaturateWheelSpeeds(stateArr, FREE_SPEED)

                io.setDesiredStates(PerCorner.fromConventionalArray(stateArr))
                Logger.recordOutput("Drivetrain/Desired States", *stateArr)
            }
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
            val discretized = ChassisSpeeds.discretize(value, Robot.period)
            val states = kinematics.toCornerSwerveModuleStates(discretized)
            moduleStates = states
        }

    // Set the drivetrain to an X-formation to passively prevent movement in any direction.
    fun brake(): Command =
        runOnce {
            // set the modules to radiate outwards from the chassis origin
            moduleStates =
                MODULE_POSITIONS.map { position -> SwerveModuleState(0.0, position.translation.angle) }
        }

    // Get the estimated pose of the drivetrain using the pose estimator.
    var estimatedPose: Pose2d
        get() = poseEstimator.estimatedPosition
        set(value) = poseEstimator.resetPosition(
            gyroRotation.toRotation2d(),
            inputs.measuredPositions.toTypedArray(),
            value
        )

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run {
            if (abs(translationJoystick.x) > JOYSTICK_DEADBAND
                || abs(translationJoystick.y) > JOYSTICK_DEADBAND
                || abs(rotationJoystick.x) > JOYSTICK_DEADBAND
            ) {
                val translationInput =
                    Translation2d(-translationJoystick.y, -translationJoystick.x).rotateBy(DRIVER_ROTATION)

                chassisSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationInput.x * FREE_SPEED.baseUnitMagnitude(),
                        translationInput.y * FREE_SPEED.baseUnitMagnitude(),
                        -rotationJoystick.x * TAU * 1.25,
                        gyroRotation.toRotation2d()
                    )
            } else {
                // set the modules to radiate outwards from the chassis origin
                moduleStates =
                    MODULE_POSITIONS.map { position -> SwerveModuleState(0.0, position.translation.angle) }
            }
        }

    fun findWheelCircumfrence(): Command =
        runOnce {
            zeroGyro()
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

    fun driveWithJoystickPointingTowards(translationJoystick: Joystick, target: Translation2d): Command {
        Logger.recordOutput("Drivetrain/Polar Driving Target", target)
        val rotationPIDController = PIDController(ROTATION_PID_GAINS).apply {
            enableContinuousInput(0.0, TAU)
        }
        return run {
            val magnitude = rotationPIDController.calculate(
                target.minus(estimatedPose.translation).angle.radians - (TAU/2),
                estimatedPose.rotation.radians
            )

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translationJoystick.y * FREE_SPEED.baseUnitMagnitude(),
                translationJoystick.x * FREE_SPEED.baseUnitMagnitude(),
                magnitude,
                gyroRotation.toRotation2d(),
            )
        }
    }

    fun zeroGyro() {
        gyroRotation = Rotation3d()
    }

    fun pathfindToPose(target: Pose2d): Command =
        AutoBuilder.pathfindToPose(target, DEFAULT_PATHING_CONSTRAINTS, 0.0)

    override val talonCANStatuses = io.modules.flatMap { it.talonCANStatuses }
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

// Constants
internal val WHEEL_BASE: Double = Units.inchesToMeters(30.0)
internal val TRACK_WIDTH: Double = Units.inchesToMeters(28.0)

internal const val JOYSTICK_DEADBAND = 0.04

internal val COMP_MODULE_POSITIONS =
    PerCorner(
        frontLeft =
        Pose2d(
            Translation2d(WHEEL_BASE, TRACK_WIDTH) / 2.0,
            Rotation2d.fromDegrees(0.0)
        ),
        backLeft =
        Pose2d(
            Translation2d(-WHEEL_BASE, TRACK_WIDTH) / 2.0,
            Rotation2d.fromDegrees(270.0)
        ),
        backRight =
        Pose2d(
            Translation2d(-WHEEL_BASE, -TRACK_WIDTH) / 2.0,
            Rotation2d.fromDegrees(0.0)
        ),
        frontRight =
        Pose2d(
            Translation2d(WHEEL_BASE, -TRACK_WIDTH) / 2.0,
            Rotation2d.fromDegrees(270.0)
        ),
    )

internal val PRACTICE_MODULE_POSITIONS =
    PerCorner(
        frontLeft =
        Pose2d(
            Translation2d(WHEEL_BASE, TRACK_WIDTH) / 2.0,
            Rotation2d.fromDegrees(-90.0)
        ),
        backLeft =
        Pose2d(
            Translation2d(-WHEEL_BASE, TRACK_WIDTH) / 2.0,
            Rotation2d.fromDegrees(0.0)
        ),
        backRight =
        Pose2d(
            Translation2d(-WHEEL_BASE, -TRACK_WIDTH) / 2.0,
            Rotation2d.fromDegrees(90.0)
        ),
        frontRight =
        Pose2d(
            Translation2d(WHEEL_BASE, -TRACK_WIDTH) / 2.0,
            Rotation2d.fromDegrees(180.0)
        ),
    )


internal val MODULE_POSITIONS = when (Robot.model) {
    Robot.Model.COMPETITION -> COMP_MODULE_POSITIONS
    Robot.Model.PRACTICE -> PRACTICE_MODULE_POSITIONS
    Robot.Model.SIMULATION -> PRACTICE_MODULE_POSITIONS
}

// Chassis Control
internal val FREE_SPEED = MetersPerSecond.of(8.132)
internal val ROTATION_SPEED = RadiansPerSecond.of(14.604)
internal val WHEEL_ODOMETRY_STD_DEV = VecBuilder.fill(0.2, 0.2, 0.005)

internal val TRANSLATION_PID_GAINS = PIDGains(0.5, 0.0, 1.0)
internal val ROTATION_PID_GAINS = PIDGains(3.0, 0.0, 0.4)

// Pathing
internal val DEFAULT_PATHING_CONSTRAINTS =
    PathConstraints(FREE_SPEED.baseUnitMagnitude(), 3.879, ROTATION_SPEED.baseUnitMagnitude(), 24.961)
internal val PATH_FOLLOWER_CONFIG = HolonomicPathFollowerConfig(
    TRANSLATION_PID_GAINS.toPPLib(),
    ROTATION_PID_GAINS.toPPLib(),
    FREE_SPEED.baseUnitMagnitude(),
    MODULE_POSITIONS.frontLeft.translation.norm,
    ReplanningConfig(true, true, Units.inchesToMeters(3.0), Units.inchesToMeters(1.5)),
)

// drive with joysticks
val DRIVER_ROTATION = when (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
    DriverStation.Alliance.Red -> Rotation2d.fromRotations(0.5)
    DriverStation.Alliance.Blue -> Rotation2d()
}

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
