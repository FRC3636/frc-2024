package com.frcteam3636.frc2024.utils.swerve

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState

enum class DrivetrainCorner {
    FRONT_LEFT, BACK_LEFT, BACK_RIGHT, FRONT_RIGHT
}

data class PerCorner<T>(
    val frontLeft: T, val backLeft: T, val backRight: T, val frontRight: T
) : Collection<T> {
    operator fun get(corner: DrivetrainCorner): T = when (corner) {
        DrivetrainCorner.FRONT_LEFT -> frontLeft
        DrivetrainCorner.BACK_LEFT -> backLeft
        DrivetrainCorner.BACK_RIGHT -> backRight
        DrivetrainCorner.FRONT_RIGHT -> frontRight
    }


    fun <U> map(block: (T) -> U): PerCorner<U> = mapWithCorner { x, _ -> block(x) }
    fun <U> mapWithCorner(block: (T, DrivetrainCorner) -> U): PerCorner<U> =
        generate { corner -> block(this[corner], corner) }


    fun <U> zip(second: PerCorner<U>): PerCorner<Pair<T, U>> = generate { corner -> Pair(this[corner], second[corner]) }

    private fun sequence(): Sequence<T> = sequenceOf(frontLeft, backLeft, backRight, frontRight)
    override fun iterator(): Iterator<T> = sequence().iterator()

    override val size: Int = 4

    override fun isEmpty(): Boolean = false

    override fun containsAll(elements: Collection<T>): Boolean = elements.all { contains(it) }

    override fun contains(element: T): Boolean = sequence().contains(element)


    companion object {
        fun <T> generate(block: (DrivetrainCorner) -> T): PerCorner<T> = PerCorner(
            frontLeft = block(DrivetrainCorner.FRONT_LEFT),
            backLeft = block(DrivetrainCorner.BACK_LEFT),
            backRight = block(DrivetrainCorner.BACK_RIGHT),
            frontRight = block(DrivetrainCorner.FRONT_RIGHT),
        )

        internal fun <T> fromConventionalArray(array: Array<T>): PerCorner<T> =
            PerCorner<T>(
                frontLeft = array[0],
                backLeft = array[1],
                backRight = array[2],
                frontRight = array[3],
            )
    }
}

fun SwerveModuleState.toTranslation2dPerSecond(): Translation2d = Translation2d(this.speedMetersPerSecond, this.angle)

fun SwerveDriveKinematics.toCornerSwerveModuleStates(speeds: ChassisSpeeds): PerCorner<SwerveModuleState> =
    PerCorner.fromConventionalArray(toSwerveModuleStates(speeds))

fun SwerveDriveKinematics.cornerStatesToChassisSpeeds(states: PerCorner<SwerveModuleState>): ChassisSpeeds =
    toChassisSpeeds(*states.toList().toTypedArray())

fun SwerveDriveKinematics(translations: PerCorner<Translation2d>) =
    SwerveDriveKinematics(*translations.toList().toTypedArray())