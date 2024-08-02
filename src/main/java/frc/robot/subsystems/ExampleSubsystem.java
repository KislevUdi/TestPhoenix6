package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Log.LogManager;

import static frc.robot.Constants.ExampleConstants.*;
import static frc.robot.Utils.*;


public class ExampleSubsystem extends SubsystemBase {

  TalonFX motor;
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  DutyCycleOut dutyCycle = new DutyCycleOut(0);

  public ExampleSubsystem() {
    motor = new TalonFX(MOTOR_ID,CAN);
    configMotor();
    addLog();
    addCommand();
  }

  private void configMotor() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.RotorToSensorRatio = 4*0.0254*Math.PI/8.14;

    cfg.CurrentLimits.SupplyCurrentLimit = 10;
    cfg.CurrentLimits.SupplyCurrentThreshold = 12;
    cfg.CurrentLimits.SupplyTimeThreshold = 0.2;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3;
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;

    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.PeakForwardDutyCycle = 0.4;
    cfg.MotorOutput.PeakReverseDutyCycle = -0.4;

    cfg.Slot0.kP = KP;
    cfg.Slot0.kI = KI;
    cfg.Slot0.kD = KD;
    cfg.Slot0.kS = KS; // add 0.5 Volts
    cfg.Slot0.kV = KV; 
    cfg.Slot0.kA = KA;

    cfg.Voltage.PeakForwardVoltage = 5;
    cfg.Voltage.PeakReverseVoltage = -5;

    motor.getConfigurator().apply(cfg);
  }

  private void addLog() {
    LogManager.addEntry("motor/position", motor::getPosition, false);
    LogManager.addEntry("motor/Velocity", motor::getVelocity, true);
    LogManager.addEntry("motor/Acceleration", motor::getAcceleration, true);
    LogManager.addEntry("motor/Voltage", motor::getMotorVoltage, true);
    LogManager.addEntry("motor/Current", motor::getStatorCurrent, true);
    LogManager.addEntry("motor/CloseLoopError", motor::getClosedLoopError, true);
    LogManager.addEntry("motor/CloseLoopOutput", motor::getClosedLoopOutput, true);
    LogManager.addEntry("motor/CloseLoopP", motor::getClosedLoopProportionalOutput, true);
    LogManager.addEntry("motor/CloseLoopI", motor::getClosedLoopIntegratedOutput, true);
    LogManager.addEntry("motor/CloseLoopD", motor::getClosedLoopDerivativeOutput, true);
    LogManager.addEntry("motor/CloseLoopFF", motor::getClosedLoopFeedForward, true);
    LogManager.addEntry("motor/CloseLoopSP", motor::getClosedLoopReference, true);
  }

  private void addCommand() {
    ShuffleboardTab table = Shuffleboard.getTab("Example");
    var e = table.add("Target Velocity",0.5).getEntry();
//    DoubleTopic dt = NetworkTableInstance.getDefault().getDoubleTopic("Target Velocity");
//    DoubleEntry e = dt.getEntry(0.5);
    Command c = new StartEndCommand(()->setVelocity(e.getDouble(0.5)), ()->setPower(0), this);
    table.add("Run",c);
    SmartDashboard.putData(c);
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
    motor.setControl(dutyCycle.withOutput(power));
  }

  public void setVelocity(double meterPerSec) {
    motor.setControl(velocityVoltage.withVelocity(meterPerSec));
  }


  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
