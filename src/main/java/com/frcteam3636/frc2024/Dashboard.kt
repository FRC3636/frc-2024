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
            val noteDetectorIpAddr = InetAddress.getByName("10.36.36.10")
            while (true) {
                try {
                    val noteDetectorOnline = noteDetectorIpAddr.isReachable(1000)
                    SmartDashboard.putBoolean("Note Detector OK", noteDetectorOnline)
                } catch (err: Exception) {
                    SmartDashboard.putBoolean("Note Detector OK", false)
                    err.printStackTrace()
                }
                Thread.sleep(1000)
            }
        }

        thread(isDaemon = true) {
            val canDiagnostics = TalonFXDiagnosticCollector(Drivetrain, Shooter.Pivot)
            while (true) {
                val talonCanStatus = canDiagnostics.tryReceivePeriodic()
                val canStatus = RobotController.getCANStatus()
                SmartDashboard.putBoolean("CAN Bus OK", (canStatus.transmitErrorCount == 0 && canStatus.receiveErrorCount == 0))
                SmartDashboard.putNumber("CAN Bus Utilization", canStatus.percentBusUtilization)
                SmartDashboard.putBoolean("CANivore Bus OK", (talonCanStatus.isOK))
                SmartDashboard.putString("CANivore Bus Status", talonCanStatus.getName())
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
