package com.frcteam3636.frc2024.subsystems.climber

import com.frcteam3636.frc2024.CANSparkMax
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.SparkAbsoluteEncoder
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ClimberIO {
    class ClimberInputs : LoggableInputs {
        var climberPosition = 0.0
        override fun toLog(table: LogTable?) {
            table?.put("Climber Position", climberPosition)
        }

        override fun fromLog(table: LogTable) {
            climberPosition = table.get("Climber Position", climberPosition)
        }
    }

    fun updateInputs(inputs: ClimberInputs)

    fun moveClimber(speed: Double)
}

class ClimberIOSim : ClimberIO {
    private var elevatorSim = ElevatorSim(
        ELEVATOR_KV,
        ELEVATOR_KA,
        DCMotor.getNEO(1),
        ELEVATOR_MIN_HEIGHT,
        ELEVATOR_MAX_HEIGHT,
        true,
        ELEVATOR_START_HEIGHT
    )

    override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
        inputs.climberPosition = elevatorSim.positionMeters
    }

    override fun moveClimber(speed: Double) {
        elevatorSim.setInputVoltage(12.0 * speed)
        elevatorSim.update(0.02)
    }

    companion object Constants {
        //TODO: Find all of these
        const val ELEVATOR_KV = 0.1
        const val ELEVATOR_KA = 0.1
        const val ELEVATOR_MAX_HEIGHT = 1.0
        const val ELEVATOR_MIN_HEIGHT = 0.0
        const val ELEVATOR_START_HEIGHT = 0.5
    }
}

class ClimberIOReal : ClimberIO {
    private var climberMotor =
        CANSparkMax(REVMotorControllerId.ClimberMotor, CANSparkLowLevel.MotorType.kBrushless)
            .apply { burnFlash() }
    private val climberEncoder =
        climberMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).apply {
            velocityConversionFactor = Units.rotationsToRadians(1.0) * CLIMBER_GEAR_RATIO / 60
            positionConversionFactor = Units.rotationsToRadians(1.0) * CLIMBER_GEAR_RATIO
        }

    override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
        inputs.climberPosition = climberEncoder.position * ELEVATOR_TRAVEL_PER_ENCODER_ROTATION
    }

    override fun moveClimber(speed: Double) {
        climberMotor.set(speed)
    }

    companion object Constants {
        const val CLIMBER_GEAR_RATIO = 1.0
        //TODO: Find this value. The current value is an estimate
        const val ELEVATOR_TRAVEL_PER_ENCODER_ROTATION = 0.003175
    }
}
