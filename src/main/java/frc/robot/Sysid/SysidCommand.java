// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Sysid;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Log.LogManager;

public class SysidCommand extends Command {
  double minPower;
  double maxPower; 
  double accelerationTime;
  DoubleConsumer setPower;
  double nextPower;
  double startTime;
  boolean isAccelerating;
  Sysid sysid;

  /** Creates a new SysidAccelerationCommand. */
  public SysidCommand(double minPower, double maxPower, double accelerationTime, DoubleConsumer setPower,Sysid sysid, Subsystem ...subsystems) {

    this.minPower = Math.min(minPower,maxPower);
    this.maxPower = Math.max(minPower,maxPower);
    this.accelerationTime = accelerationTime;
    this.setPower = setPower;
    this.sysid = sysid;
    addRequirements(subsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sysid.startCollecting();
    isAccelerating = true;
    nextPower = minPower;
    startTime = RobotController.getFPGATime() / 1000.0;
    LogManager.log("Starting accelearation - " + minPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calcNextPower();
    setPower.accept(nextPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    setPower.accept(0);
    sysid.stopCollecting();
    sysid.calculate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return nextPower == 0;
  }

  private void calcNextPower() {
    double time =  RobotController.getFPGATime() / 1000.0;
    double change = (maxPower - minPower)*(time - startTime)/accelerationTime;
    if(isAccelerating) {
      nextPower = minPower + change;
      if(nextPower > maxPower) {
        nextPower = maxPower;
        isAccelerating = false;
        startTime = time;
        LogManager.log("Starting deaccelearation - " + maxPower);
      }
    } else if(nextPower > minPower) {
      nextPower = maxPower - change;
      if(nextPower < minPower) {
        nextPower = minPower;
      }
    } else {
      LogManager.log("end deaccelearation - " + minPower);
      nextPower = 0;
    }
  }
}
