// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Log;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Log.LogManager.LogEntry;

/** Add your docs here. */
public class ChassisSpeedLogEntry {
    String name;
    LogEntry vx;
    LogEntry vy;
    LogEntry omega;
    public ChassisSpeedLogEntry(String name) {
        this.name = name;
        vx = LogManager.getEntry(name + "/vx");
        vy = LogManager.getEntry(name + "/vy");
        omega = LogManager.getEntry(name + "/omega");
    }

    public void log(ChassisSpeeds chassisSpeed) {
        vx.log(chassisSpeed.vxMetersPerSecond);
        vy.log(chassisSpeed.vxMetersPerSecond);
        omega.log(chassisSpeed.omegaRadiansPerSecond);
    }

}
