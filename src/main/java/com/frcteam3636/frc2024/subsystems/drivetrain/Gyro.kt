package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Quaternion
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.PI
import kotlin.math.sign

interface Gyro {
    var rotation: Rotation3d

    fun periodic() {}
}

class GyroNavX(private var offset: Rotation3d = Rotation3d(0.0, 0.0, PI)) : Gyro {
    private val ahrs = AHRS()

    override var rotation: Rotation3d
        get() =
            offset +
                    Rotation3d(
                        Quaternion(
                            ahrs.quaternionW.toDouble(),
                            ahrs.quaternionX.toDouble(),
                            ahrs.quaternionY.toDouble(),
                            ahrs.quaternionZ.toDouble()
                        )
                    )
        set(value) {
            offset = value - rotation
        }
}

class GyroSim(private val modules: PerCorner<SwerveModule>) : Gyro {
    override var rotation = Rotation3d()

    override fun periodic() {
        val moduleVelocities =
            modules.map { Translation2d(it.state.speedMetersPerSecond, it.state.angle) }
        val translationVelocity = moduleVelocities.reduce(Translation2d::plus) / 4.0
        val rotationalVelocities = moduleVelocities.map { it - translationVelocity }
        val yawVelocity =
            sign(rotationalVelocities.frontLeft.y) * rotationalVelocities.frontLeft.norm /
                    MODULE_POSITIONS.frontLeft.translation.norm

        rotation += Rotation3d(0.0, 0.0, yawVelocity) * Robot.period
    }
}
