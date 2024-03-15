package com.frcteam3636.frc2024.subsystems.shooter

import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

interface FeederIO {
    class Inputs : LoggableInputs {
        override fun toLog(table: LogTable?) {

        }

        override fun fromLog(table: LogTable?) {

        }

    }

    fun updateInputs(inputs: Inputs)

    fun setIndexerVoltage(voltage: Measure<Voltage>)
}

class FeederIOReal : FeederIO {

    private val indexer =
        CANSparkFlex(REVMotorControllerId.Indexer, CANSparkLowLevel.MotorType.kBrushless)


    override fun updateInputs(inputs: FeederIO.Inputs) {

    }

    override fun setIndexerVoltage(voltage: Measure<Voltage>) {
        indexer.setVoltage(voltage.baseUnitMagnitude())
        Logger.recordOutput("Shooter/Flywheels/Indexer Voltage", voltage)
    }

}

class FeederIOSim : FeederIO {
    override fun updateInputs(inputs: FeederIO.Inputs) {

    }

    override fun setIndexerVoltage(voltage: Measure<Voltage>) {

    }
}