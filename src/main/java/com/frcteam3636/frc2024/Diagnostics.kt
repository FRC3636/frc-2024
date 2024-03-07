package com.frcteam3636.frc2024

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

fun testSequence(tests: List<Pair<String, Command>>, controller: XboxController): Command {
    return Commands.sequence(
        Commands.runOnce({
            SmartDashboard.putString("Current Test", "Press A to start")
            SmartDashboard.putString("Next Test", tests.first().first)
        }),
        Commands.waitUntil { controller.aButtonPressed },
        *tests.asIterable().mapIndexed { i: Int, it: Pair<String, Command> ->
            Commands.deadline(
                Commands.waitUntil { controller.aButtonPressed },
                Commands.print("Running test: ${it.first}"),
                Commands.runOnce({
                    SmartDashboard.putString("Current Test", it.first)
                    SmartDashboard.putString("Next Test", tests.map { it.first }.getOrElse(i + 1) { "None" })
                }),
                it.second,
            )
        }.toTypedArray(),
        Commands.runOnce({
            SmartDashboard.putString("Current Test", "None")
        })
    )
}

val tests: List<Pair<String, Command>> = listOf(
//    "Drive Translation Static" to Drivetrain.translationStaticTest(),
//    "Drive Translation Dynamic" to Drivetrain.translationDynamicTest(),
    "Drive Rotation" to Drivetrain.rotationStaticTest(),
)

interface TalonFXStatusProvider {
    val talonCANStatuses: List<StatusSignal<*>>
}

class TalonFXDiagnosticCollector(vararg providers: TalonFXStatusProvider) {
    private val diagnostics = providers.flatMap { it.talonCANStatuses }
    private val timeout = Units.Seconds.of(0.5)

    fun tryReceivePeriodic(): Boolean {
        val result = BaseStatusSignal.waitForAll(timeout.baseUnitMagnitude(), *diagnostics.toTypedArray())
        return result == StatusCode.OK
    }
}