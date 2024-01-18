package com.frcteam3636.frc2024.subsystems.drivetrain

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Quaternion
import edu.wpi.first.math.geometry.Rotation3d


interface Gyro {
    var rotation: Rotation3d

    fun periodic() {}
}


class GyroNavX(private var offset: Rotation3d = Rotation3d()) : Gyro {
    private val ahrs = AHRS()

    override var rotation: Rotation3d
        get() = Rotation3d(
            Quaternion(
                ahrs.quaternionW.toDouble(),
                ahrs.quaternionX.toDouble(),
                ahrs.quaternionY.toDouble(),
                ahrs.quaternionZ.toDouble()
            )
        )
        set(value) {
            val currentRotation = Rotation3d(
                Quaternion(
                    ahrs.quaternionW.toDouble(),
                    ahrs.quaternionX.toDouble(),
                    ahrs.quaternionY.toDouble(),
                    ahrs.quaternionZ.toDouble()
                )
            )
            offset = currentRotation - rotation
        }
}

class GyroSim : Gyro {
    override var rotation = Rotation3d()
}