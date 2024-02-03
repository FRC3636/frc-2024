package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.utils.math.MotorFFGains
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.frcteam3636.frc2024.utils.math.pidGains
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

interface FlywheelIO {
    class Inputs : LoggableInputs {
        var leftSpeed = Rotation2d()
        var rightSpeed = Rotation2d()

        override fun toLog(table: LogTable) {
            table.put("Left Speed", leftSpeed)
            table.put("Right Speed", rightSpeed)
        }

        override fun fromLog(table: LogTable) {
            leftSpeed = table.get("Left Speed", leftSpeed)[0]
            rightSpeed = table.get("Right Speed", rightSpeed)[0]
        }
    }

    fun updateInputs(inputs: Inputs)

    /** Set the speeds of the flywheels in rad/s. */
    fun setSpeeds(leftSpeed: Double, rightSpeed: Double)
}

class FlywheelIOReal : FlywheelIO {
    private val leftSpark =
        CANSparkMax(REVMotorControllerId.LeftShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()

            inverted = true
            encoder.velocityConversionFactor = Units.rotationsPerMinuteToRadiansPerSecond(1.0) * GEAR_RATIO

            pidController.apply {
                pidGains = PID_GAINS
                ff = FF_GAINS.v
            }
        }

    private val rightSpark =
        CANSparkMax(REVMotorControllerId.RightShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()

            inverted = false
            encoder.velocityConversionFactor = Units.rotationsPerMinuteToRadiansPerSecond(1.0) * GEAR_RATIO

            pidController.apply {
                pidGains = PID_GAINS
                ff = FF_GAINS.v
            }
        }

    override fun updateInputs(inputs: FlywheelIO.Inputs) {
        inputs.leftSpeed = Rotation2d(leftSpark.encoder.velocity)
        inputs.rightSpeed = Rotation2d(rightSpark.encoder.velocity)
    }

    override fun setSpeeds(leftSpeed: Double, rightSpeed: Double) {
        // set the onboard PIDF controllers to the desired speeds
        leftSpark.pidController.setReference(leftSpeed, CANSparkBase.ControlType.kVelocity)
        rightSpark.pidController.setReference(rightSpeed, CANSparkBase.ControlType.kVelocity)

        // log setpoints as outputs
        Logger.recordOutput("Shooter/Flywheels/Left Setpoint", leftSpeed)
        Logger.recordOutput("Shooter/Flywheels/Right Setpoint", rightSpeed)
    }

    internal companion object {
        const val GEAR_RATIO = 1.0

        val PID_GAINS = PIDGains()
        val FF_GAINS = MotorFFGains(v = 0.0)
    }
}

class FlywheelIOSim : FlywheelIO {
    // simulating the flywheels wouldn't actually allow us to test any more,
    // since we're just using onboard SPARK MAX feedback controllers
    private var leftSpeed = Rotation2d()
    private var rightSpeed = Rotation2d()

    override fun updateInputs(inputs: FlywheelIO.Inputs) {
        inputs.leftSpeed = leftSpeed
        inputs.rightSpeed = rightSpeed
    }

    override fun setSpeeds(leftSpeed: Double, rightSpeed: Double) {
        this.leftSpeed = Rotation2d(leftSpeed)
        this.rightSpeed = Rotation2d(rightSpeed)

        // log setpoints as outputs
        Logger.recordOutput("Shooter/Flywheels/Left Setpoint", leftSpeed)
        Logger.recordOutput("Shooter/Flywheels/Right Setpoint", rightSpeed)
    }
}