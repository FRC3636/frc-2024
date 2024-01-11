package com.frcteam3636.frc2024

enum class CANDevice(val id: Int) {

    //swerve
    FrontLeftTurningMotor(6),
    FrontLeftDrivingMotor(5),

    FrontRightTurningMotor(4),
    FrontRightDrivingMotor(3),

    BackLeftTurningMotor(2),
    BackLeftDrivingMotor(1),

    BackRightTurningMotor(8),
    BackRightDrivingMotor(7),

    IndexerMotor(13),

    TurretMotor(10),

    FlywheelMotor(11),
    ShooterFeedMotor(12),

    BallIntakeRollerMotor(15),
    BallIntakeArmMotor(14),
}