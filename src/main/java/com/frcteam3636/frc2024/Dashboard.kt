package com.frcteam3636.frc2024

import com.ctre.phoenix6.StatusCode
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.subsystems.shooter.Shooter
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.net.InetAddress
import kotlin.concurrent.thread

object Dashboard {
    init {
        thread(isDaemon = true) {
            val visionPcAddr = InetAddress.getByName("10.36.36.10")
            while (true) {
                try {
                    val visionPcOnline = visionPcAddr.isReachable(1000)
                    SmartDashboard.putBoolean("Vision PC OK", visionPcOnline)
                } catch (err: Exception) {
                    SmartDashboard.putBoolean("Vision PC OK", false)
                    err.printStackTrace()
                }
                Thread.sleep(1000)
            }
        }

        thread(isDaemon = true) {
            val canDiagnostics = TalonFXDiagnosticCollector(Drivetrain, Shooter.Pivot)
            while (true) {
                val canStatus = canDiagnostics.tryReceivePeriodic()
                SmartDashboard.putBoolean("CAN Bus OK", (canStatus == StatusCode.OK && (RobotController.getCANStatus().transmitErrorCount < 0 || RobotController.getCANStatus().receiveErrorCount < 0)))
                SmartDashboard.putString("CAN Status", canStatus.getName())
                Thread.sleep(250)
            }
        }
    }

    fun update() {
        SmartDashboard.putBoolean("Battery Full", RobotController.getBatteryVoltage() >= 12.3)
        SmartDashboard.putBoolean("NavX OK", Drivetrain.gyroConnected)
        SmartDashboard.putBoolean("All Cameras OK", Drivetrain.allCamerasConnected)
    }
}
