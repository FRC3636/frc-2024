package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

interface FlywheelIO {
    class Inputs : LoggableInputs {
        var leftSpeed = RadiansPerSecond.zero()
        var rightSpeed = RadiansPerSecond.zero()
        var leftVoltage: Double = 0.0
        var rightVoltage: Double = 0.0
        var leftCurrent = Amps.zero()
        var rightCurrent = Amps.zero()
        var leftPos = Radians.zero()
        var rightPos = Radians.zero()

        override fun toLog(table: LogTable) {
            table.put("Left Speed", leftSpeed)
            table.put("Right Speed", rightSpeed)
            table.put("left Current", leftCurrent)
            table.put("Right current", rightCurrent)
        }

        override fun fromLog(table: LogTable) {
            leftSpeed = table.get("Left Speed", leftSpeed)
            rightSpeed = table.get("Right Speed", rightSpeed)
            leftCurrent = table.get("left Current", leftCurrent)
            rightCurrent = table.get("Right current", rightCurrent)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun setFlywheelVoltage(left: Measure<Voltage>, right: Measure<Voltage>) {}
}

class FlywheelIOReal : FlywheelIO {
    private val leftSpark =
        CANSparkFlex(REVMotorControllerId.LeftShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()

            inverted = false
            encoder.apply {
                positionConversionFactor = Units.rotationsToRadians(1.0)
                velocityConversionFactor = Units.rotationsPerMinuteToRadiansPerSecond(1.0)
            }
        }

    private val rightSpark =
        CANSparkFlex(REVMotorControllerId.RightShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()

            inverted = false
            encoder.apply {
                positionConversionFactor = Units.rotationsToRadians(1.0)
                velocityConversionFactor = Units.rotationsPerMinuteToRadiansPerSecond(1.0)
            }
        }


    override fun updateInputs(inputs: FlywheelIO.Inputs) {
        inputs.leftSpeed = RadiansPerSecond.of(leftSpark.encoder.velocity)
        inputs.rightSpeed = RadiansPerSecond.of(rightSpark.encoder.velocity)

        inputs.leftVoltage = leftSpark.busVoltage * leftSpark.appliedOutput
        inputs.rightVoltage = rightSpark.busVoltage * rightSpark.appliedOutput

        inputs.leftCurrent = Amps.of(leftSpark.outputCurrent)
        inputs.rightCurrent = Amps.of(rightSpark.outputCurrent)

        inputs.leftPos = Radians.of(leftSpark.encoder.position)
        inputs.rightPos = Radians.of(rightSpark.encoder.position)
    }

    override fun setFlywheelVoltage(left: Measure<Voltage>, right: Measure<Voltage>) {
        leftSpark.setVoltage(left.baseUnitMagnitude())
        rightSpark.setVoltage(right.baseUnitMagnitude())

        Logger.recordOutput("Shooter/Flywheels/Left Effort", left)
        Logger.recordOutput("Shooter/Flywheels/Right Effort", right)
    }
}

class FlywheelIOSim : FlywheelIO {
    // simulating the flywheels wouldn't actually allow us to test any more,
    // since we're just using onboard SPARK MAX feedback controllers
    private val leftFlywheel = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, 0.01)
    private val rightFlywheel = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, 0.01)

    override fun updateInputs(inputs: FlywheelIO.Inputs) {
        inputs.leftSpeed = RadiansPerSecond.of(leftFlywheel.angularVelocityRadPerSec)
        inputs.rightSpeed = RadiansPerSecond.of(rightFlywheel.angularVelocityRadPerSec)
    }

    override fun setFlywheelVoltage(left: Measure<Voltage>, right: Measure<Voltage>) {
        leftFlywheel.setInputVoltage(left.baseUnitMagnitude())
        rightFlywheel.setInputVoltage(right.baseUnitMagnitude())

        Logger.recordOutput("Shooter/Flywheels/Left Effort", left)
        Logger.recordOutput("Shooter/Flywheels/Right Effort", right)
    }
}