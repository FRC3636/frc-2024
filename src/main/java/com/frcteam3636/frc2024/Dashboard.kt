package com.frcteam3636.frc2024

import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import java.net.InetAddress
import kotlin.concurrent.thread

object Dashboard {
    private val autoChooser = AutoBuilder.buildAutoChooser("Middle 2 Piece")
        .apply {
            SmartDashboard.putData(this)
        }
    val currentAuto: Command
        get() = autoChooser.selected

    init {
        thread {
            val visionPcAddr = InetAddress.getByName("10.36.36.10")
            while (true) {
                val visionPcOnline = visionPcAddr.isReachable(1000)
                SmartDashboard.putBoolean("Vision PC Online", visionPcOnline)
                Thread.sleep(1000)
            }
        }
    }

    fun update() {
        SmartDashboard.putBoolean("Battery Ready", RobotController.getBatteryVoltage() >= 12.3)
        SmartDashboard.putBoolean("NavX Connected", Drivetrain.gyroConnected)
    }
}