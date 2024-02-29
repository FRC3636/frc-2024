package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.math.PIDController
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.frcteam3636.frc2024.utils.math.TAU
import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.frcteam3636.frc2024.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.frc2024.utils.swerve.toCornerSwerveModuleStates
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
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.math.abs

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
            val stateArr = value.toTypedArray()
            SwerveDriveKinematics.desaturateWheelSpeeds(stateArr, FREE_SPEED)

            io.setDesiredStates(PerCorner.fromConventionalArray(stateArr))
            Logger.recordOutput("Drivetrain/DesiredStates", *stateArr)
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
    fun brake(): Command =
        runOnce {
            // set the modules to radiate outwards from the chassis origin
            moduleStates =
                MODULE_POSITIONS.map { position -> SwerveModuleState(0.0, position.translation.angle) }
        }

    // Get the estimated pose of the drivetrain using the pose estimator.
    val estimatedPose: Pose2d
        get() = poseEstimator.estimatedPosition

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run {
            if (abs(translationJoystick.x) > JOYSTICK_DEADBAND
                || abs(translationJoystick.y) > JOYSTICK_DEADBAND
                || abs(rotationJoystick.x) > JOYSTICK_DEADBAND
            ) {
                chassisSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        -translationJoystick.y * FREE_SPEED,
                        -translationJoystick.x * FREE_SPEED,
                        -rotationJoystick.x * TAU * 8,
                        gyroRotation.toRotation2d()
                    )
            } else {
                // set the modules to radiate outwards from the chassis origin
                moduleStates =
                    MODULE_POSITIONS.map { position -> SwerveModuleState(0.0, position.translation.angle) }
            }
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

class DrivetrainIOReal(override val modules: PerCorner<out SwerveModule>) : DrivetrainIO() {
    override val gyro = GyroNavX()
}

class DrivetrainIOSim : DrivetrainIO() {
    override val modules = PerCorner.generate { SimSwerveModule() }
    override val gyro = GyroSim(modules.map { it })
}

// Constants
internal val WHEEL_BASE: Double = Units.inchesToMeters(13.0)
internal val TRACK_WIDTH: Double = Units.inchesToMeters(14.0)

internal val ROTATION_PID_CONTROLLER = PIDController(PIDGains(0.3, 0.0, 0.0))
internal val FREE_SPEED = 17.0
internal val JOYSTICK_DEADBAND = 0.04

internal val MODULE_POSITIONS =
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
