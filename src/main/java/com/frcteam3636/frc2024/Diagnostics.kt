package com.frcteam3636.frc2024

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import edu.wpi.first.units.Units

interface TalonFXStatusProvider {
    val talonCANStatuses: List<StatusSignal<*>>
}

class TalonFXDiagnosticCollector(vararg providers: TalonFXStatusProvider) {
    private val diagnostics = providers.flatMap { it.talonCANStatuses }
    private val timeout = Units.Seconds.of(0.5)

    fun tryReceivePeriodic(): StatusCode {
        return if (diagnostics.isNotEmpty()) {
            try {
                BaseStatusSignal.waitForAll(timeout.baseUnitMagnitude(), *diagnostics.toTypedArray())
            } catch (e: Exception) {
                StatusCode.GeneralError
            }
        } else {
            StatusCode.OK
        }
    }
}
