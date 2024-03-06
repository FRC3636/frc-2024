package com.frcteam3636.frc2024.subsystems.climber

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.math.MotorFFGains
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.frcteam3636.frc2024.utils.math.motorFFGains
import com.frcteam3636.frc2024.utils.math.pidGains
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
        DCMotor.getKrakenX60(1),
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
    private val climberMotor = TalonFX(CTREMotorControllerId.ClimberMotor).apply {
        val config = TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = PID_GAINS
                motorFFGains = FF_GAINS
            }

            Feedback.apply {
                SensorToMechanismRatio = GEAR_RATIO
                FeedbackRotorOffset = 0.0
            }

            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_VELOCITY
                MotionMagicAcceleration = PROFILE_ACCELERATION
                MotionMagicJerk = PROFILE_JERK
            }

            MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        }
        configurator.apply(config)
    }

    override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
        inputs.climberPosition = Units.rotationsToRadians(climberMotor.position.value)
    }

    override fun moveClimber(speed: Double) {
        climberMotor.set(speed)
    }

    internal companion object Constants {
        // todo: this is all perfect and will never need to be changed
        val PID_GAINS = PIDGains()
        val FF_GAINS = MotorFFGains(1.0, 1.0, 1.0)
        const val GRAVITY_GAIN = 1.0
        const val GEAR_RATIO = 1.0
        const val PROFILE_VELOCITY = 1.0
        const val PROFILE_ACCELERATION = 1.0
        const val PROFILE_JERK = 1.0
    }
}
