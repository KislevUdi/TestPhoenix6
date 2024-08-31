// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Kinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

/** 
 * Chassis Speed
 * 
 * The next cycle velocity as a 2d vector
 * The path radians per second
 * The chassis rad per second
 */
public class ChassisSpeed {
    private Translation2d velocity; // relative to robot or field
    private double pathRadPerSec; // path RadPerSec
    private double chassisRadPerSec; // chassis rotation rate

    public static ChassisSpeed fieldToRobot(ChassisSpeed speed, Rotation2d heading) {
        return new ChassisSpeed(speed.velocity.rotateBy(heading.unaryMinus()), speed.pathRadPerSec, speed.chassisRadPerSec);
    }
    public static ChassisSpeed robotToField(ChassisSpeed speed, Rotation2d heading) {
        return new ChassisSpeed(speed.velocity.rotateBy(heading), speed.pathRadPerSec, speed.chassisRadPerSec);
    }

    public ChassisSpeed(Translation2d velocity, double pathRadPerSec, double chassisRadPerSec) {
        this.velocity = velocity;
        this.pathRadPerSec = pathRadPerSec;
        this.chassisRadPerSec = chassisRadPerSec;
    }
    public ChassisSpeed(Translation2d velocity, double radPerSec) {
        this(velocity, 0, radPerSec);
    }
    public ChassisSpeed(double vx, double vy, double radPerSec) {
        this(new Translation2d(vx, vy), 0, radPerSec);
    }

    public Translation2d getVelocity() {
        return velocity;
    }

    public double getVX() {
        return velocity.getX();
    }
    public double getPathRadPerSec() {
        return pathRadPerSec;
    }
    public void setPathRadPerSec(double pathRadPerSec) {
        this.pathRadPerSec = pathRadPerSec;
    }
    public double getChassisRadPerSec() {
        return chassisRadPerSec;
    }
    public void setChassisRadPerSec(double chassisRadPerSec) {
        this.chassisRadPerSec = chassisRadPerSec;
    }

    /**
     * Set the radius
     * @param radius - in meter, positiove value result in positive pathRadPerSec
     */
    public void setPathRadius(double radius) {
        if(Math.abs(radius) < 0.1) {
            pathRadPerSec = 0;
        }
        pathRadPerSec = velocity.getNorm() * Robot.kDefaultPeriod / radius;
    }
    public double getVY() {
        return velocity.getY();
    }
    /**
     * Calculate the position/heading change from current pose in dt seconds
     * @param dtSeconds - delta time in seconds
     * @return
     */
    public Pose2d poseChange(double dtSeconds) {
        double rads = pathRadPerSec*dtSeconds/2;
        return new Pose2d(velocity.rotateBy(new Rotation2d(rads)), new Rotation2d(chassisRadPerSec*dtSeconds));
    }

    /**
     * update chassis speed to next cycle
     * the velocity direction need to change base on the path and chassis rate of turn
     */
    public void updateToNextCycle() {
        double rad = (pathRadPerSec-chassisRadPerSec)*Robot.kDefaultPeriod;
        velocity = velocity.rotateBy(new Rotation2d(rad));
    }
}
