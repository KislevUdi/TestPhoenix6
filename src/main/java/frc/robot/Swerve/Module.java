// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Log.LogManager;
import frc.robot.Log.LogManager.LogEntry;
import frc.robot.Util.TalonMotor;

/** Add your docs here. */
public class Module {

    private String name;
    private Translation2d positionRelativeToRobotCenter;   
    private TalonMotor steer;
    private TalonMotor drive;
    private CANcoder absEncoder;
    LogEntry setVelocityLogger;
    LogEntry setSteerDirectionLogger;
    LogEntry setSteerTargetDirectionLogger;

    public Module(ModuleConfig config) {
        name = config.name;
        positionRelativeToRobotCenter = config.positionRelativeToRobotCenter;
        steer = new TalonMotor(config.steerConfig);
        drive = new TalonMotor(config.driveConfig);
        absEncoder = new CANcoder(config.absEncoderID, config.absEncodedrCanbus);
        absEncoder.getAbsolutePosition().setUpdateFrequency(200);
        setVelocityLogger = LogManager.getEntry(name + "/setVelocity");
        setSteerDirectionLogger = LogManager.getEntry(name + "/setDirection");
        setSteerDirectionLogger = LogManager.getEntry(name + "/setTargetDirection");
        LogManager.addEntry(name + "/absEncoder", absEncoder::getAbsolutePosition);
    }

    public double getDriveVelocity() {
        return drive.getCurrentVelocity();
    }

    public Rotation2d getSteerDirection() {
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getSteerDirection());
    }

    public Translation2d getPosition() {
        return positionRelativeToRobotCenter;
    }

    public void setDirection(Rotation2d direction) {
        double current = steer.getCurrentPosition();
        double target = current + direction.minus(getSteerDirection()).getDegrees();
        steer.setMotorPosition(target);
        setSteerDirectionLogger.log(direction.getDegrees());
        setSteerTargetDirectionLogger.log(target);
    }

    public void setVelocity(double velocity) {
        drive.setVelocity(velocity);
        setVelocityLogger.log(velocity);
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getSteerDirection());
        setVelocity(optimized.speedMetersPerSecond);
        if(optimized.speedMetersPerSecond == 0) {
            setDirection(getSteerDirection());
        } else {
            setDirection(optimized.angle);
        }

    }

    public void periodic() {
        // update state
    }
    
}
