package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANDevice
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel


class ShooterIOReal : ShooterIO {

    private val flywheelMotor = CANSparkFlex(CANDevice.FlywheelMotor.id, CANSparkLowLevel.MotorType.kBrushless)

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs?) {
        TODO()
    }

}