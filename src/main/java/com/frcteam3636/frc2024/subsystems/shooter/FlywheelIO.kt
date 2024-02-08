package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.utils.math.MotorFFGains
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.frcteam3636.frc2024.utils.math.pidGains
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import java.sql.ResultSetMetaData

interface FlywheelIO {
    class Inputs : LoggableInputs {
        var leftSpeed = RadiansPerSecond.zero()
        var rightSpeed = RadiansPerSecond.zero()
        var leftVoltage: Double = 0.0
        var rightVoltage: Double = 0.0
        var leftPos = Radians.zero()
        var rightPos = Radians.zero()

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

    fun setIndexerVoltage(volts: Measure<Voltage>) {}

    fun setVoltage(left: Measure<Voltage>, right: Measure<Voltage>) {}
}

class FlywheelIOReal : FlywheelIO {
    private val leftSpark =
        CANSparkFlex(REVMotorControllerId.LeftShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()

            inverted = false
            encoder.apply {
                positionConversionFactor = Units.rotationsToRadians(1.0)
                velocityConversionFactor = Units.rotationsPerMinuteToRadiansPerSecond(1.0) * GEAR_RATIO
            }
        }

    private val rightSpark =
        CANSparkFlex(REVMotorControllerId.RightShooterFlywheel, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()

            inverted = false
            encoder.apply {
                positionConversionFactor = Units.rotationsToRadians(1.0)
                velocityConversionFactor = Units.rotationsPerMinuteToRadiansPerSecond(1.0) * GEAR_RATIO
            }
        }

    private val indexer =
        CANSparkFlex(REVMotorControllerId.Indexer, CANSparkLowLevel.MotorType.kBrushless)


    override fun updateInputs(inputs: FlywheelIO.Inputs) {
        inputs.leftSpeed = RadiansPerSecond.of(leftSpark.encoder.velocity)
        inputs.rightSpeed = RadiansPerSecond.of(rightSpark.encoder.velocity)
        inputs.leftVoltage = leftSpark.busVoltage * leftSpark.appliedOutput
        inputs.rightVoltage = rightSpark.busVoltage * rightSpark.appliedOutput
        inputs.leftPos = Radians.of(leftSpark.encoder.position)
        inputs.rightPos = Radians.of(rightSpark.encoder.position)
    }

    override fun setIndexerVoltage(volts: Measure<Voltage>) {
        indexer.setVoltage(volts.baseUnitMagnitude())
    }

    override fun setVoltage(left: Measure<Voltage>, right: Measure<Voltage>) {
        leftSpark.setVoltage(left.baseUnitMagnitude())
        rightSpark.setVoltage(right.baseUnitMagnitude())

        Logger.recordOutput("Shooter/Flywheels/Left Effort", left)
        Logger.recordOutput("Shooter/Flywheels/Right Effort", right)
    }



    internal companion object {
        const val GEAR_RATIO = 1.0
    }
}

class FlywheelIOSim : FlywheelIO {
    // simulating the flywheels wouldn't actually allow us to test any more,
    // since we're just using onboard SPARK MAX feedback controllers
    private var leftSpeed = RadiansPerSecond.zero()
    private var rightSpeed = RadiansPerSecond.zero()

    override fun updateInputs(inputs: FlywheelIO.Inputs) {
        inputs.leftSpeed = leftSpeed
        inputs.rightSpeed = rightSpeed
    }

    override fun setVoltage(left: Measure<Voltage>, right: Measure<Voltage>) {
        TODO()
    }
}