
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.TalonMotor;

import static frc.robot.Constants.ExampleConstants.*;
import static frc.robot.Util.Utils.*;


public class ExampleSubsystem extends SubsystemBase {

  TalonMotor motor;
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  DutyCycleOut dutyCycle = new DutyCycleOut(0);
  MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

  public ExampleSubsystem() {
    motor = new TalonMotor(MOTOR_CONFIG);
    addCommand();
  }

  private void addCommand() {
    ShuffleboardTab table = Shuffleboard.getTab("Example");
    var e = table.add("Target Velocity",0.5).getEntry();
    var e2 = table.add("Target Position",1).getEntry();
    Command c = (new StartEndCommand(()->setVelocity(e.getDouble(0.5)), ()->setPower(0), this)).withTimeout(3);
    table.add("Run Velocity",c);
    c = (new StartEndCommand(()->setPosition(e2.getDouble(0.5)), ()->setPower(0), this)).withTimeout(5);
    table.add("Run Position",c);
  }

  public double getPosition() {
    return motor.getPosition().getValue();
  }
  public double getVelocity() {
    return motor.getVelocity().getValue();
  }
  public double getAcceleration() {
    return motor.getAcceleration().getValue();
  }
  public double getMotorVoltage() {
    return motor.getMotorVoltage().getValue();
  }
  public double getMotorCurrent() {
    return motor.getStatorCurrent().getValue();
  }
  public double getClosedLoopError() {
    return getDouble(motor.getClosedLoopError());
  }
  public double getClosedLoopOutput() {
    return getDouble(motor.getClosedLoopOutput());
  }
  public double getClosedLoopPOutput() {
    return getDouble(motor.getClosedLoopProportionalOutput());
  }
  public double getClosedLoopDOutput() {
    return getDouble(motor.getClosedLoopDerivativeOutput());
  }
  public double getClosedLoopIOutput() {
    return getDouble(motor.getClosedLoopIntegratedOutput());
  }
  public double getClosedLoopFFOutput() {
    return getDouble(motor.getClosedLoopFeedForward());
  }
  public double getClosedLoopSP() {
    return getDouble(motor.getClosedLoopReference());
  }
  public boolean getMotorForwardSwitch() {
    return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }
  public boolean getMotorReverseSwicth() {
    return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public ControlModeValue getMotorMode() {
    return motor.getControlMode().getValue();
  }
   
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
  public void setPower(double power) {
    motor.setDuty(power);
  }

  public void setVelocity(double meterPerSec) {
    motor.setVelocity(meterPerSec);
  }

  public void setPosition(double position) {
    motor.setMotorPosition(position);
  }


  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
