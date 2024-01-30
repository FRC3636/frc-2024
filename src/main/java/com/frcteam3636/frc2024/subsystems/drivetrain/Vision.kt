package com.frcteam3636.frc2024.subsystems.drivetrain

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import java.util.*


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
                if (result.targetsUsed.first().bestCameraToTarget.translation.norm > APRIL_TAG_DISTANCE_FILTER.baseUnitMagnitude()
                    || result.targetsUsed.first().poseAmbiguity > APRIL_TAG_DISTANCE_FILTER.baseUnitMagnitude()
                ) {
                    Optional.empty<VisionMeasurement>()
                }

                if (result.estimatedPose.x < 0 || result.estimatedPose.x > FIELD_LENGTH.baseUnitMagnitude() ||
                    result.estimatedPose.y < 0 || result.estimatedPose.y > FIELD_WIDTH.baseUnitMagnitude()
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

internal val ODOMETRY_STD_DEV: Matrix<N3, N1> = VecBuilder.fill(0.2, 0.2, 0.005)
internal val APRIL_TAG_DISTANCE_FILTER = FIELD_LENGTH.divide(2.0)
internal const val APRIL_TAG_AMBIGUITY_FILTER = 0.3
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

