// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Log;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Log.LogManager.LogEntry;

/** Add your docs here. */
public class Pose2dLogger extends LogEntry {
    Supplier<Pose2d> supplier;
    LogEntry[] entries;


    public Pose2dLogger(String name, Supplier<Pose2d> supplier ) {
        super(name, null, null, false, 5);
        this.supplier = supplier;
        entries = new LogEntry[] {
            LogManager.getEntry(name + "/x")
            ,LogManager.getEntry(name + "/y")
            ,LogManager.getEntry(name + "/heading")
        };
    }

    @Override
    void log() {
        if(level < LogManager.logLevel )
            return;
        Pose2d p = supplier.get();
        Translation2d t = p.getTranslation();
        Rotation2d r = p.getRotation();
        LogManager.log(entries,LogManager.time(),t.getX(), t.getY(), MathUtil.inputModulus(r.getDegrees(), -180, 180));
    }

} 
