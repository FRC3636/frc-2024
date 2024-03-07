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
        thread(isDaemon = true) {
            val visionPcAddr = InetAddress.getByName("10.36.36.10")
            while (true) {
                try {
                    val visionPcOnline = visionPcAddr.isReachable(1000)
                    SmartDashboard.putBoolean("Vision PC Connected", visionPcOnline)
                } catch (err: Exception) {
                    SmartDashboard.putBoolean("Vision PC Connected", false)
                    err.printStackTrace()
                }
                Thread.sleep(1000)
            }
        }
    }

    fun update() {
        SmartDashboard.putBoolean("Battery Full", RobotController.getBatteryVoltage() >= 12.3)
        SmartDashboard.putBoolean("NavX OK", Drivetrain.gyroConnected)
        SmartDashboard.putBoolean("All Cameras OK", Drivetrain.allCamerasConnected)
    }
}