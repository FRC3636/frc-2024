package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

interface FlywheelIO {
    class Inputs : LoggableInputs {
        var leftSpeed = MutableMeasure.zero(RadiansPerSecond)
        var rightSpeed = MutableMeasure.zero(RadiansPerSecond)
        var leftVoltage = MutableMeasure.zero(Volts)
        var rightVoltage = MutableMeasure.zero(Volts)
        var leftPos = MutableMeasure.zero(Radians)
        var rightPos = MutableMeasure.zero(Radians)

        override fun toLog(table: LogTable) {
            table.put("Left Speed", leftSpeed)
            table.put("Right Speed", rightSpeed)
        }

        override fun fromLog(table: LogTable) {
            leftSpeed = table.get("Left Speed", leftSpeed)
            rightSpeed = table.get("Right Speed", rightSpeed)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun setIndexerVoltage(voltage: Measure<Voltage>)

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

    private val indexer =
        CANSparkFlex(REVMotorControllerId.Indexer, CANSparkLowLevel.MotorType.kBrushless)


    override fun updateInputs(inputs: FlywheelIO.Inputs) {
        inputs.leftSpeed.mut_setMagnitude(leftSpark.encoder.velocity)
        inputs.rightSpeed.mut_setMagnitude(rightSpark.encoder.velocity)
        inputs.leftVoltage.mut_setMagnitude(leftSpark.busVoltage * leftSpark.appliedOutput)
        inputs.rightVoltage.mut_setMagnitude( rightSpark.busVoltage * rightSpark.appliedOutput)
        inputs.leftPos.mut_setMagnitude(leftSpark.encoder.position)
        inputs.rightPos.mut_setMagnitude(rightSpark.encoder.position)
    }

    override fun setIndexerVoltage(voltage: Measure<Voltage>) {
        indexer.setVoltage(voltage.baseUnitMagnitude())

        Logger.recordOutput("Shooter/Flywheels/Indexer Voltage", voltage)
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
        inputs.leftSpeed.mut_setMagnitude(leftFlywheel.angularVelocityRadPerSec)
        inputs.rightSpeed.mut_setMagnitude(rightFlywheel.angularVelocityRadPerSec)
    }

    override fun setFlywheelVoltage(left: Measure<Voltage>, right: Measure<Voltage>) {
        leftFlywheel.setInputVoltage(left.baseUnitMagnitude())
        rightFlywheel.setInputVoltage(right.baseUnitMagnitude())

        Logger.recordOutput("Shooter/Flywheels/Left Effort", left)
        Logger.recordOutput("Shooter/Flywheels/Right Effort", right)
    }

    override fun setIndexerVoltage(voltage: Measure<Voltage>) {
        // no-op, we don't have an indexer in the sim

        Logger.recordOutput("Shooter/Flywheels/Indexer Voltage", voltage)
    }
}