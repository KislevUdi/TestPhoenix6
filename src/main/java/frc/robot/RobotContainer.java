package frc.robot;

import frc.robot.Log.LogManager;
import frc.robot.Log.LogManager.LogEntry;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
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

  public Translation2d vel = new Translation2d();
  public Translation2d pos = new Translation2d();
  public BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  LogEntry[] logEntries = LogManager.getLogEntries("accel","ax","ay","az","vx","vy","px","py");
  double lastTime = 0;
  public void specialPeriodic() {
    if(lastTime == 0) {
      lastTime = Timer.getFPGATimestamp();
    } else {
      double ax = accelerometer.getX();
      double ay = accelerometer.getY();
      double az = accelerometer.getZ();
      Translation2d a = new Translation2d(ax, ay);
      double dt = Timer.getFPGATimestamp() - lastTime;
      lastTime += dt;
      vel = vel.plus(a.times(dt));
      pos = pos.plus(vel.times(dt)).plus(a.times(dt*dt/2));
      long t = (long)(lastTime * 1000);
      LogManager.log(logEntries,t,ax,ay,az,vel.getX(),vel.getY(),pos.getX(),pos.getY());
    }
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
