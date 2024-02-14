package com.frcteam3636.frc2024.subsystems.drivetrain

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonUtils
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*


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
            targetIDs = table.get("Target IDs Used", targetIDs.toIntArray())!!.toList()
        }
    }

    abstract fun updateInputs(inputs: Inputs)
}

class AprilTagPhotonVisionIOReal(private val name: String, private val cameraTransform: Transform3d) :
    AprilTagPhotonVisionIO() {
    override val camera = PhotonCamera(name).apply {
        driverMode = false
        pipelineIndex = PipelineIndex.AprilTag.index
    }

    val fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)

    override val poseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
        fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraTransform
    )

    override fun updateInputs(inputs: Inputs) {


        result.map {

            inputs.targetCorners = it.targetsUsed.flatMap { target ->
                target.detectedCorners
            }.map { corner ->
                Translation2d(corner.x, corner.y)
            }

            inputs.targetIDs = it.targetsUsed.map { target ->
                target.fiducialId
            }
        }
    }

}


abstract class ObjectDetectorIO {

    abstract val camera: PhotonCamera

    class Inputs : LoggableInputs {
        var objectLocations: List<Translation2d> = mutableListOf()
        var timestampSecond: Double = 0.0

        override fun toLog(table: LogTable?) {
            table?.put("Objects Detected", *objectLocations.toTypedArray())
            table?.put("Timestamp", timestampSecond)
        }

        override fun fromLog(table: LogTable?) {
            objectLocations = table?.get("Objects Detected", *objectLocations.toTypedArray())!!.toList()
            timestampSecond = table.get("Timestamp", timestampSecond)
        }
    }

    abstract fun updateInputs(inputs: Inputs)
}

class ObjectDetectorIOReal(private val name: String, private val tranform: Transform3d) : ObjectDetectorIO() {
    override val camera = PhotonCamera(name).apply {
        driverMode = false
        pipelineIndex = PipelineIndex.ObjectDetection.index
    }

    override fun updateInputs(inputs: Inputs) {

        val result = camera.latestResult
        inputs.objectLocations = result.targets.map { target ->
            getTranslationToTarget(target)
        }
        inputs.timestampSecond = result.timestampSeconds
    }

    private fun getTranslationToTarget(target: PhotonTrackedTarget): Translation2d {
        val distance = PhotonUtils.calculateDistanceToTargetMeters(
            this.tranform.z,
            NOTE_HEIGHT_METERS,
            tranform.rotation.y,
            0.0,
        )

        //negate yaw to convert from CV conventions to math conventions (counterclockwise increases angle)
        val cameraToTarget: Translation2d =
            PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-target.yaw))

        val robotToTarget = cameraToTarget.minus(tranform.translation.toTranslation2d())

        return robotToTarget

    }
}

enum class PipelineIndex(val index: Int) {
    AprilTag(0), ObjectDetection(1)
}

internal val NOTE_HEIGHT_METERS = 0.0