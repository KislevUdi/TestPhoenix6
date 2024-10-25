// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Log;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Log.LogManager.LogEntry;

/** Add your docs here. */
public class Rotation2dLogger extends LogEntry {
    Supplier<Rotation2d> supplier;
    LogEntry[] entries;


    public Rotation2dLogger(String name, Supplier<Rotation2d> supplier ) {
        super(name, null, null, false, 5);
        this.supplier = supplier;
        entries = new LogEntry[] {
            LogManager.getEntry(name + "/degrees")
        };
    }

    @Override
    void log() {
        if(level < LogManager.logLevel )
            return;
        Rotation2d t = supplier.get();
        LogManager.log(entries,LogManager.time(),t.getDegrees());
    }

} 
