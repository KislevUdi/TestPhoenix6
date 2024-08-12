
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Log.LogManager;
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
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  DutyCycleOut dutyCycle = new DutyCycleOut(0);
  MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

  public ExampleSubsystem() {
    steerMotor = new TalonMotor(STEER_MOTOR_CONFIG);
    driveMotor = new TalonMotor(DRIVE_MOTOR_CONFIG);
    addCommand();
    LogManager.log("example subsystem initialized");
  }

  private void addCommand() {
    ShuffleboardTab table = Shuffleboard.getTab("Example");
    var driveVelocityEntry = table.add("Target Drive Velocity", 0.5).getEntry();
    var drivePositionEntry = table.add("Target Drive Position", 1).getEntry();
    var steerVelocityEntry = table.add("Target Steer Velocity", 120).getEntry();
    var steerPositionEntry = table.add("Target Steer Position", 90).getEntry();
    Command c = getUserCommand(this::setDriveVelocity, driveVelocityEntry, 0.5, new WaitCommand(3), this::setDrivePower);
    table.add("Drive Velocity", c);
    c = getUserCommand(this::setDrivePosition, drivePositionEntry, 2, getTriggerCommand(this::getDrivePosition,drivePositionEntry,0.1,5), this::setDrivePower);
    table.add("Drive Position", c);
    c = getUserCommand(this::setSteerVelocity, steerVelocityEntry, 120, new WaitCommand(3), this::setSteerPower);
    table.add("Steer Velocity", c);
    c = getUserCommand(this::setSteerPosition, steerPositionEntry, 90, getTriggerCommand(this::getSteerPosition,steerPositionEntry,5,5), this::setSteerPower);
    table.add("Steer Position", c);
  }


  private Command getUserCommand(DoubleConsumer set, GenericEntry entry, double defaultValue, Command waitCommand, DoubleConsumer setStop) {
    return new InstantCommand(()->set.accept(entry.getDouble(defaultValue)), this)
      .andThen(waitCommand, new InstantCommand(()->setStop.accept(0), this));
  }
  private Command getTriggerCommand(DoubleSupplier getPosition, GenericEntry targetPosition, double maxError, double time) {
      return new WaitUntilCommand(()->(Math.abs(getPosition.getAsDouble() - targetPosition.getDouble(0)) < maxError)).raceWith(new WaitCommand(time));
  }

  
  /** 
   * @return double
   */
  public double getSteerPosition() {
    return steerMotor.getPosition().getValue();
  }
  /** 
   * @return double
   */
  public double getDrivePosition() {
    return driveMotor.getPosition().getValue();
  }
  /** 
   * @return double
   */
  public double getSteerVelocity() {
    return steerMotor.getVelocity().getValue();
  }
  /** 
   * @return double
   */
  public double getDriveVelocity() {
    return steerMotor.getVelocity().getValue();
  }

  public void setSteerVoltage(double volts) {
    steerMotor.setVoltage(volts);
  }
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  public void setSteerPower(double power) {
    steerMotor.setDuty(power);
  }
  public void setDrivePower(double power) {
    driveMotor.setDuty(power);
  }

  public void setSteerVelocity(double meterPerSec) {
    steerMotor.setVelocity(meterPerSec);
  }
  public void setDriveVelocity(double meterPerSec) {
    driveMotor.setVelocity(meterPerSec);
  }

  public void setSteerPosition(double position) {
    steerMotor.setMotorPosition(position);
  }
  public void setDrivePosition(double position) {
    driveMotor.setMotorPosition(position);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
