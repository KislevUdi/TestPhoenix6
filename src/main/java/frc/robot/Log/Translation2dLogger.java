// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Log;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Log.LogManager.LogEntry;

/** Add your docs here. */
public class Translation2dLogger extends LogEntry {
    Supplier<Translation2d> supplier;
    LogEntry[] entries;


    public Translation2dLogger(String name, Supplier<Translation2d> supplier ) {
        super(name, null, null, false, 5);
        this.supplier = supplier;
        entries = new LogEntry[] {
            LogManager.getEntry(name + "/x")
            ,LogManager.getEntry(name + "/y")
        };
    }

    @Override
    void log() {
        if(level < LogManager.logLevel )
            return;
        Translation2d t = supplier.get();
        LogManager.log(entries,LogManager.time(),t.getX(), t.getY());
    }

} 
