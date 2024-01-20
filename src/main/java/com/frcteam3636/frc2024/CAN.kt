package com.frcteam3636.frc2024.can

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax

// This module contains one enum for each (device type, manufacturer) pair we use.

enum class REVMotorControllerId(val num: Int) {
    FrontLeftTurningMotor(1),
    BackLeftTurningMotor(2),
    BackRightTurningMotor(3),
    FrontRightTurningMotor(4),
    LeftShooterFlywheel(0),
    RightShooterFlywheel(0),
    OverTheBumperIntakeArm(0),
    OverTheBumperIntakeFeed(0),
    UnderTheBumperIntakeRoller(0),
    ClimberMotor(0),

}

fun CANSparkMax(id: REVMotorControllerId, type: CANSparkLowLevel.MotorType) = CANSparkMax(id.num, type)
fun CANSparkFlex(id: REVMotorControllerId, type: CANSparkLowLevel.MotorType) = CANSparkFlex(id.num, type)

enum class CTREMotorControllerId(val num: Int, val bus: String) {
    FrontLeftDrivingMotor(1, "*"),
    BackLeftDrivingMotor(2, "*"),
    BackRightDrivingMotor(3, "*"),
    FrontRightDrivingMotor(4, "*"),
}

fun TalonFX(id: CTREMotorControllerId) = TalonFX(id.num, id.bus)