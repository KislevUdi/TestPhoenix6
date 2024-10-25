// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Log;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class LoggedChassisSpeeds extends ChassisSpeeds {

  public LoggedChassisSpeeds(String name, double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
    super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    LogManager.addEntry(name + "/vX", ()->vxMetersPerSecond);
    LogManager.addEntry(name + "/vX", ()->vyMetersPerSecond);
    LogManager.addEntry(name + "/vX", ()->omegaRadiansPerSecond);
  }

  public void set(ChassisSpeeds chassisSpeeds) {
    vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond;
    vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond;
    omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond;
  }

  public Translation2d getVelocityVector() {
    return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
  }

  public double getVelocity() {
    return Math.hypot(vxMetersPerSecond, vyMetersPerSecond);
  }

  public Rotation2d getHeading() {
    return getVelocityVector().getAngle();
  }

  public void setVelocity(Translation2d velocity, double omega) {
    vxMetersPerSecond = velocity.getX();
    vyMetersPerSecond = velocity.getY();
    omegaRadiansPerSecond = omega;
  }
  public void setVelocity(Translation2d velocity) {
    setVelocity(velocity,0);
  }

  public void setOmega(double omega) {
    omegaRadiansPerSecond = omega;
  }

  public void setOmega(Rotation2d omega) {
    omegaRadiansPerSecond = omega.getRadians();
  }

  public void add(ChassisSpeeds chassisSpeeds) {
    vxMetersPerSecond += chassisSpeeds.vxMetersPerSecond;
    vyMetersPerSecond += chassisSpeeds.vyMetersPerSecond;
    omegaRadiansPerSecond += chassisSpeeds.omegaRadiansPerSecond;
  }
  public void substract(ChassisSpeeds chassisSpeeds) {
    vxMetersPerSecond -= chassisSpeeds.vxMetersPerSecond;
    vyMetersPerSecond -= chassisSpeeds.vyMetersPerSecond;
    omegaRadiansPerSecond -= chassisSpeeds.omegaRadiansPerSecond;
  }
  public void mult(double mult) {
    vxMetersPerSecond *= mult;
    vyMetersPerSecond *= mult;
    omegaRadiansPerSecond *= mult;
  }

  public void reverse() {
    vxMetersPerSecond = -vxMetersPerSecond;
    vyMetersPerSecond = -vyMetersPerSecond;
    omegaRadiansPerSecond = -omegaRadiansPerSecond;
  }

}

