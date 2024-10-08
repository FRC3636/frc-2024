package com.frcteam3636.frc2024

import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.subsystems.shooter.Shooter
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.net.InetAddress
import kotlin.concurrent.thread

object Dashboard {
    private val limelightAddrs = ArrayList<String>()
    private var limelightsConnected = true

    init {
        limelightAddrs.add("10.36.36.11") // TODO: Think of a better way to do this

        thread(isDaemon = true) {
            while (true) {
                var allConnected = true // Prevent the value from flickering
                for (addr in limelightAddrs) {
                    try {
                        InetAddress.getByName(addr).isReachable(1000)
                    } catch (err: Exception) {
                        allConnected = false
                        break
                    }
                }
                limelightsConnected = allConnected
                Thread.sleep(2000)
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
                SmartDashboard.putBoolean("Pivot Encoder OK", Shooter.Pivot.encoderConnected)
                Thread.sleep(250)
            }
        }
    }

    fun update() {
        SmartDashboard.putBoolean("Battery Full", RobotController.getBatteryVoltage() >= 12.3)
        SmartDashboard.putBoolean("NavX OK", Drivetrain.gyroConnected)
        SmartDashboard.putBoolean("All Cameras OK", limelightsConnected && Drivetrain.allCamerasConnected)
    }
}
