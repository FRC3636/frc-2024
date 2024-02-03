package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.Robot
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Subsystem
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

    private val mechanism = Mechanism2d(3.0, 3.0, BLACK)
    private val mechanismRoot = mechanism.getRoot("Shooter", 0.5, 0.5)
    private val armLigament = mechanismRoot.append(
        MechanismLigament2d(
            "Arm", 2.0, 0.0, 10.0, WHITE,
        )
    )
    private val flywheelLigament = armLigament.append(
        MechanismLigament2d(
            "Flywheel", 0.25, 0.0, 5.0, BLUE
        )
    )

    init {
        SmartDashboard.putData("Shooter", mechanism)
    }

    override fun periodic() {
        flywheelIO.updateInputs(flywheelInputs)
        Logger.processInputs("Shooter/Flywheels", flywheelInputs)

        pivotIO.updateInputs(pivotInputs)
        Logger.processInputs("Shooter/Pivot", pivotInputs)

        armLigament.angle = pivotInputs.position.degrees
        flywheelLigament.color = if (flywheelInputs.leftSpeed.rotations > 1.0) {
            BLUE
        } else {
            WHITE
        }
    }
}

internal val BLACK = Color8Bit("#0a0a0a")
internal val WHITE = Color8Bit("#ffffff")
internal val BLUE = Color8Bit("#1d48a3")
