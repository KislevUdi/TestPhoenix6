// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Log;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Log.LogManager.LogEntry;

/** Add your docs here. */
public class ChassisSpeedsLogger extends LogEntry {
    Supplier<ChassisSpeeds> supplier;
    LogEntry[] entries;


    public ChassisSpeedsLogger(String name, Supplier<ChassisSpeeds> supplier ) {
        super(name, null, null, false, 5);
        this.supplier = supplier;
        entries = new LogEntry[] {
            LogManager.getEntry(name + "/vx")
            ,LogManager.getEntry(name + "/vy")
            ,LogManager.getEntry(name + "/omega")
        };
    }

    @Override
    void log() {
        if(level < LogManager.logLevel )
            return;
        ChassisSpeeds speeds = supplier.get();
        LogManager.log(entries,LogManager.time(), speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond);
    }

} 
