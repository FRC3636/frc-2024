package com.frcteam3636.frc2024.subsystems.drivetrain

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator


interface AbsolutePoseIO {
    class Inputs : LoggableInputs {
        var absolutePoseMeasurement: AbsolutePoseMeasurement = AbsolutePoseMeasurement(Pose3d(), 0.0, Translation3d())

        override fun toLog(table: LogTable) {
            table.put("Pose", absolutePoseMeasurement.pose)
            table.put("Timestamp", absolutePoseMeasurement.timestamp)
            table.put("Standard Deviations", absolutePoseMeasurement.stdDeviation)
        }

        override fun fromLog(table: LogTable?) {
            val pose = table?.get("Pose", Pose3d())!![0]
            val timestamp = table.get("Timestamp", 0.0)
            val stdDeviation = table.get("Standard Deviations", Translation3d())!![0]

            absolutePoseMeasurement = AbsolutePoseMeasurement(pose, timestamp, stdDeviation)
        }
    }

    fun updateInputs(inputs: Inputs)
}

// Singular. Lime. Light. A.P.ril T.a . I. Nout. Real. (on friend)
class SLLAPTIR(val name: String, val chassisToCamera: Transform3d) : AbsolutePoseIO {
    override fun updateInputs(inputs: AbsolutePoseIO.Inputs) {
        TODO("Not yet implemented")
    }
}

// Plural. Photon. Phision.
class PPPAPTIR(val name: String, val chassisToCamera: Transform3d) : AbsolutePoseIO {

    val estimator: PhotonPoseEstimator = PhotonPoseEstimator(
        APRIL_TAG_FIELD_LAYOUT,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonCamera(name).apply { driverMode = false },
        chassisToCamera
    )

    override fun updateInputs(inputs: AbsolutePoseIO.Inputs) {

        estimator.update().ifPresent {
            val pose = it.estimatedPose
            val timestamp = it.timestampSeconds
            val stdDev = PHOTON_APRIL_TAG_STD_DEV.of(pose.translation.norm, it.targetsUsed.size)

            inputs.absolutePoseMeasurement = AbsolutePoseMeasurement(pose, timestamp, stdDev)
        }
    }
}

data class AbsolutePoseMeasurement(val pose: Pose3d, val timestamp: Double, val stdDeviation: Translation3d);

fun SwerveDrivePoseEstimator.addAbsolutePoseMeasurement(measurement: AbsolutePoseMeasurement) {
    addVisionMeasurement(
        measurement.pose.toPose2d(),
        measurement.timestamp,
        measurement.stdDeviation.toStdDeviationMatrix()
    )
}

fun Translation3d.toStdDeviationMatrix(): Matrix<N3, N1> {
    return VecBuilder.fill(x, y, z)
}

fun interface StandardDeviation {
    abstract fun of(distance: Double, numTags: Int): Translation3d
}

internal val APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)


//TODO make the functoin on friend
internal val PHOTON_APRIL_TAG_STD_DEV = StandardDeviation { distance, num -> Translation3d() }