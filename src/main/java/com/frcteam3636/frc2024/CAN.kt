package com.frcteam3636.frc2024

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

    // fixme: these can ids should probably be updated in hardware because 15 and 7 are pretty
    // random
    LeftShooterFlywheel(15),
    RightShooterFlywheel(7),

    // todo: the following `3x` CAN ids are placeholders
    OverTheBumperIntakeArm(31),
    OverTheBumperIntakeFeed(32),
    UnderTheBumperIntakeRoller(33),
    ClimberMotor(34),
}

fun CANSparkMax(id: REVMotorControllerId, type: CANSparkLowLevel.MotorType) =
    CANSparkMax(id.num, type)

fun CANSparkFlex(id: REVMotorControllerId, type: CANSparkLowLevel.MotorType) =
    CANSparkFlex(id.num, type)

enum class CTREMotorControllerId(val num: Int, val bus: String) {
    FrontLeftDrivingMotor(1, "*"),
    BackLeftDrivingMotor(2, "*"),
    BackRightDrivingMotor(3, "*"),
    FrontRightDrivingMotor(4, "*"),
}

fun TalonFX(id: CTREMotorControllerId) = TalonFX(id.num, id.bus)
