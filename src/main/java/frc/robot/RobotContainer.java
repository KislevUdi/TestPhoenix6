package frc.robot;

import frc.robot.Log.LogManager;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  public final ExampleSubsystem exampleSubsystem;
  public final LogManager logManager;

  public RobotContainer() {
    logManager = new LogManager();
    exampleSubsystem = new ExampleSubsystem();
    configureBindings();
    LogManager.log("robot container initalized");
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
