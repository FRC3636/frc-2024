package com.frcteam3636.frc2024.subsystems.drivetrain

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.RobotBase
import java.util.*


data class VisionPoseMeasurement(val stddev: Matrix<N3, N1>, val pose: Pose3d, val timestamp: Double)
data class VisionObjectMeasurement(val objects: List<Translation2d>, val timestamp: Double)

interface VisionBackend {
    fun getMeasurement(): Optional<VisionPoseMeasurement>
}

class PhotonObjectDetectionCamera(name: String, cameraTransform: Transform3d) {
    private val io = if (RobotBase.isReal()) {
        ObjectDetectorIOReal(name, cameraTransform)
    } else {
        throw NotImplementedError("Simulated Photon Vision not implemented")
    }
    private val inputs = ObjectDetectorIO.Inputs()

    fun periodic() {
        io.updateInputs(inputs)
    }

    fun getMeasurement(): Optional<VisionObjectMeasurement> {
        val timestamp = inputs.timestampSecond

        return if (inputs.objectLocations.isEmpty()) {
            Optional.empty<VisionObjectMeasurement>()
        } else {
            Optional.of(VisionObjectMeasurement(inputs.objectLocations, timestamp))
        }

    }

}

class PhotonAprilTagCamera(name: String, cameraTransform: Transform3d) : VisionBackend {
    private val io = if (RobotBase.isReal()) {
        AprilTagPhotonVisionIOReal(name, cameraTransform)
    } else {
        throw NotImplementedError("Simulated Photon Vision not implemented")
    }
    private val inputs = AprilTagPhotonVisionIO.Inputs()

    fun periodic() {
        io.updateInputs(inputs)
    }

    override fun getMeasurement(): Optional<VisionPoseMeasurement> {
        return io.result.flatMap { result ->
            if (result.targetsUsed.first().bestCameraToTarget.translation.norm > APRIL_TAG_DISTANCE_FILTER.baseUnitMagnitude() || result.targetsUsed.first().poseAmbiguity > APRIL_TAG_DISTANCE_FILTER.baseUnitMagnitude()) {
                Optional.empty<VisionPoseMeasurement>()
            }
            if (result.estimatedPose.x < 0 || result.estimatedPose.x > FIELD_LENGTH.baseUnitMagnitude() || result.estimatedPose.y < 0 || result.estimatedPose.y > FIELD_WIDTH.baseUnitMagnitude()) {

                Optional.empty<VisionPoseMeasurement>()
            }
            Optional.of(
                VisionPoseMeasurement(
                    APRIL_TAG_STD_DEV(result.targetsUsed.first().bestCameraToTarget.x, result.targetsUsed.size),
                    result.estimatedPose,
                    result.timestampSeconds
                )
            )
        }
    }
}


internal val ODOMETRY_STD_DEV: Matrix<N3, N1> = VecBuilder.fill(0.2, 0.2, 0.005)
internal val APRIL_TAG_DISTANCE_FILTER = FIELD_LENGTH.divide(2.0)
internal const val APRIL_TAG_AMBIGUITY_FILTER = 0.3
internal val APRIL_TAG_STD_DEV = { distance: Double, count: Int ->
    val distanceMultiplier = Math.pow(distance - ((count - 1) * 2), 2.0)
    val translationalStdDev = (0.05 / (count)) * distanceMultiplier + 0.0
    val rotationalStdDev = 0.2 * distanceMultiplier + 0.1
    VecBuilder.fill(
        translationalStdDev, translationalStdDev, rotationalStdDev
    )
}

