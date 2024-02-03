package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.math.PIDController
import com.frcteam3636.frc2024.utils.math.PIDGains
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.littletonrobotics.junction.Logger

object Shooter : Subsystem {
    private val flywheelIO: FlywheelIO = when (Robot.model) {
        Robot.Model.SIMULATION -> FlywheelIOSim()
        Robot.Model.COMPETITION, Robot.Model.PRACTICE -> FlywheelIOReal()
    }
    private val flywheelInputs = FlywheelIO.Inputs()

    private val pivotIO: PivotIO = when (Robot.model) {
        Robot.Model.SIMULATION -> PivotIOSim()
        Robot.Model.COMPETITION -> PivotIOKraken()
        Robot.Model.PRACTICE -> TODO()
    }
    private val pivotInputs = PivotIO.Inputs()

    override fun periodic() {
        flywheelIO.updateInputs(flywheelInputs)
        Logger.processInputs("Shooter/Flywheels", flywheelInputs)

        pivotIO.updateInputs(pivotInputs)
        Logger.processInputs("Shooter/Pivot", pivotInputs)
    }
}
