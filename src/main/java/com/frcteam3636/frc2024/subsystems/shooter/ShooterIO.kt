package com.frcteam3636.frc2024.subsystems.shooter

import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.hardware.TalonFX
import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs

interface ShooterIO {
    class ShooterIOInputs : LoggableInputs {
        var leftSpeed = Rotation2d()
        var rightSpeed = Rotation2d()
        var position = Rotation2d()
        var pivotAngularVelocity = Rotation2d()
        var pivotAcceleration = Rotation2d()

        override fun toLog(table: LogTable) {
            table.put("Left Speed", leftSpeed)
            table.put("Right Speed", rightSpeed)
            table.put("Position", position)
        }

        override fun fromLog(table: LogTable) {
            leftSpeed = table.get("Left Speed", leftSpeed)[0]
            rightSpeed = table.get("Right Speed", rightSpeed)[0]
            position = table.get("Position", position)!![0]
        }
    }

    /** Updates the set of loggable inputs. */
    fun updateInputs(inputs: ShooterIOInputs) {}

    /** Run the launcher flywheel at the specified percent speed. */
    fun shoot(speed: Double, spin: Boolean) {}

    /** Run the launcher flywheels in reverse to intake at the specified percent speed. */
    fun intake(speed: Double) {}

    fun setPivotControlRequest(control: ControlRequest) {}
}

class ShooterIOReal : ShooterIO {

    companion object {
        const val FLYWHEEL_GEAR_RATIO = 1.0
        const val PIVOT_GEAR_RATIO = 1 / 90.0
    }



    private val left =
            CANSparkMax(
                            REVMotorControllerId.LeftShooterFlywheel,
                            CANSparkLowLevel.MotorType.kBrushless
                    )
                    .apply {
                        inverted = true
                        encoder.velocityConversionFactor =
                                Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO / 60
                    }

    private val right =
            CANSparkMax(
                            REVMotorControllerId.RightShooterFlywheel,
                            CANSparkLowLevel.MotorType.kBrushless
                    )
                    .apply {
                        encoder.velocityConversionFactor =
                                Units.rotationsToRadians(1.0) * FLYWHEEL_GEAR_RATIO / 60
                        inverted = false
                    }

    val pivotMotorConfigs = TalonFXConfiguration()
    .apply{
       
        Feedback.SensorToMechanismRatio = 1/90.0
        
    }
    
    val slot0Configs = pivotMotorConfigs.Slot0
        .apply { 
            kS = 0.0
            kV = 0.0
            kA = 0.0
            kG = 0.0
            kP = 0.0
            kI = 0.0
            kD = 0.0
        }
    
    val motionProfileConfigs: MotionMagicConfigs = pivotMotorConfigs.MotionMagic
        .apply {
            MotionMagicAcceleration = 0.0
            MotionMagicJerk = 4000.0
        }

    private val pivotLeftKraken = TalonFX(CTREMotorControllerId.LeftPivotMotor)
        .apply { 
            configurator.apply(pivotMotorConfigs)
         }
         
    private val pivotRightKraken = TalonFX(CTREMotorControllerId.RightPivotMotor)
        .apply { 
            configurator.apply(pivotMotorConfigs)
         }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        inputs.leftSpeed = Rotation2d(left.encoder.velocity)
        inputs.rightSpeed = Rotation2d(right.encoder.velocity)

        inputs.position = Rotation2d(pivotLeftKraken.position.value * PIVOT_GEAR_RATIO)
        inputs.pivotAngularVelocity = Rotation2d(pivotLeftKraken.velocity.value * PIVOT_GEAR_RATIO)
        inputs.pivotAcceleration = Rotation2d(pivotLeftKraken.acceleration.value * PIVOT_GEAR_RATIO)
    }

    override fun shoot(speed: Double, spin: Boolean) {
        left.set(speed)
        Logger.recordOutput("Shooter/Left Power", speed)
        val rSpeed =
                speed *
                        if (spin) {
                            0.75
                        } else {
                            1.0
                        }
        right.set(rSpeed)
        Logger.recordOutput("Shooter/Right Power", rSpeed)
    }

    override fun setPivotControlRequest(control: ControlRequest) {
        pivotLeftKraken.setControl(control)
    }

    override fun intake(speed: Double) {
        left.set(-speed)
        right.set(-speed)
        Logger.recordOutput("Shooter/Left Power", -speed)
        Logger.recordOutput("Shooter/Right Power", -speed)
    }
}
