
package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Log.LogManager;
import frc.robot.Sysid.Sysid;
import frc.robot.Util.TalonMotor;

import static frc.robot.Constants.ExampleConstants.*;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

  /** 
   * Example subsystem with 2 motors
   */
public class ExampleSubsystem extends SubsystemBase {

  TalonMotor steerMotor;
  TalonMotor driveMotor;

  public ExampleSubsystem() {
    steerMotor = new TalonMotor(STEER_MOTOR_CONFIG);
    driveMotor = new TalonMotor(DRIVE_MOTOR_CONFIG);
    addCommand();
    LogManager.log("example subsystem initialized");
  }

  /**
   * Add commands
   * Set velocity/position/power for bothy motors
   * Set Sysid for both motors
   */
  private void addCommand() {
    ShuffleboardTab table = Shuffleboard.getTab("Example");
    var driveVelocityEntry = table.add("Target Drive Velocity", 0.5).getEntry();
    var drivePositionEntry = table.add("Target Drive Position", 1).getEntry();
    var steerVelocityEntry = table.add("Target Steer Velocity", 120).getEntry();
    var steerPositionEntry = table.add("Target Steer Position", 90).getEntry();
    var drivePowerEntry = table.add("Drive Power'", 0.15).getEntry();
    var steerPowerEntry = table.add("Drive Power'", 0.15).getEntry();
    Command c = getUserCommand(this::setDriveVelocity, driveVelocityEntry, 0.5, new WaitCommand(3), this::setDrivePower);
    table.add("Drive Velocity", c);
    c = getUserCommand(this::setDrivePosition, drivePositionEntry, 2, getTriggerCommand(this::getDrivePosition,drivePositionEntry,0.1,5), this::setDrivePower);
    table.add("Drive Position", c);
    c = getUserCommand(this::setSteerVelocity, steerVelocityEntry, 120, new WaitCommand(3), this::setSteerPower);
    table.add("Steer Velocity", c);
    c = getUserCommand(this::setSteerPosition, steerPositionEntry, 90, getTriggerCommand(this::getSteerPosition,steerPositionEntry,5,5), this::setSteerPower);
    table.add("Steer Position", c);
    c = getUserCommand(this::setSteerPower, steerPowerEntry, 0.15, new WaitCommand(3), this::setSteerPowerAndReportVelocity);
    table.add("Steer Power", c);
    c = getUserCommand(this::setDrivePower, drivePowerEntry, 0.15, new WaitCommand(3), this::setDrivePowerAndReportVelocity);
    table.add("Drive Power", c);
    c = new Sysid(driveMotor.name()).getCommand(this::setDrivePower,0.1, 0.4, 0.5, this);
    table.add("Sysid Drive", c);
    c = new Sysid(steerMotor.name()).getCommand(this::setSteerPower,0.1, 0.4, 0.5, this);
    table.add("Sysid Steer", c);
  }

/**
 * get a user command to read the data from network table
 * call the set 
 * wait command
 * set pwer to 0
 * 
 * @param set - set the motor (power, velocity or position)
 * @param entry - Network Table entry to read the value
 * @param defaultValue - default data - required for network table read
 * @param waitCommand - wait command before stopping motor
 * @param setStop - set power command - use to set power to 0
 * @return
 */
  private Command getUserCommand(DoubleConsumer set, GenericEntry entry, double defaultValue, Command waitCommand, DoubleConsumer setStop) {
    return new InstantCommand(()->set.accept(entry.getDouble(defaultValue)), this)
      .andThen(waitCommand, new InstantCommand(()->setStop.accept(0), this));
  }

  /**
   * function that return a command that wait untill the position is in range
   * 
   * @param getPosition - function to return the current position
   * @param targetPosition - network table entry to read the required target position
   * @param maxError - max allower error
   * @param time - max wait time
   * @return
   */
  private Command getTriggerCommand(DoubleSupplier getPosition, GenericEntry targetPosition, double maxError, double time) {
      return new WaitUntilCommand(()->(Math.abs(getPosition.getAsDouble() - targetPosition.getDouble(0)) < maxError)).raceWith(new WaitCommand(time));
  }

  
  /** 
   * @return double - steer position in degrees
   */
  public double getSteerPosition() {
    return steerMotor.getPosition().getValue();
  }
  /** 
   * @return double - retrun drive positin in meters
   */
  public double getDrivePosition() {
    return driveMotor.getPosition().getValue();
  }
  /** 
   * @return double - get steer velocity in degrees per sec
   */
  public double getSteerVelocity() {
    return steerMotor.getVelocity().getValue();
  }
  /** 
   * @return double - get drive velocity in meter per sec
   */
  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValue();
  }

  /**
  * set steer voltage
  * @param volts
  */
  public void setSteerVoltage(double volts) {
    steerMotor.setVoltage(volts);
  }
  /**
  * set drive voltage
  * @param volts
  */
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  /**
  * set steer power -1 to 1
  * @param power
  */
  public void setSteerPower(double power) {
    steerMotor.setDuty(power);
  }

  /**
  * set drive power -1 to 1
  * @param power
  */
  public void setDrivePower(double power) {
    driveMotor.setDuty(power);
  }

  /**
  * set Drive power -1 to 1 and log the current velocity
  * @param power
  */
  public void setDrivePowerAndReportVelocity(double power) {
    LogManager.log(driveMotor.name() + " Velocity=" + getDriveVelocity());
    setDrivePower(power);
  }

  /**
  * set steer power -1 to 1 and log the current velocity
  * @param power
  */
  public void setSteerPowerAndReportVelocity(double power) {
    LogManager.log(steerMotor.name() + " Velocity=" + getSteerVelocity());
    setSteerPower(power);
  }

  /**
  * set steer velocity in derees per sec
  * @param degreesPerSec
  */
  public void setSteerVelocity(double degreesPerSec) {
    steerMotor.setVelocity(degreesPerSec);
  }
  /**
  * set drive velocity in meters per sec
  * @param meterPerSec
  */
  public void setDriveVelocity(double meterPerSec) {
    driveMotor.setVelocity(meterPerSec);
  }

  /**
  * set steer position in degrees (not normalizing)
  * @param degrees
  */
  public void setSteerPosition(double degrees) {
    steerMotor.setMotorPosition(degrees);
  }
  /**
  * set drive position in meters (absoulte)
  * @param positionInMeter
  */
  public void setDrivePosition(double positionInMeter) {
    driveMotor.setMotorPosition(positionInMeter);
  }
  
  /**
  * set drive position in meters - relative to current position
  * @param positionInMeter
  */
  public void setRelativeDrivePosition(double positionInMeter) {
    driveMotor.setMotorPosition(positionInMeter + driveMotor.getCurrentPosition());
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
