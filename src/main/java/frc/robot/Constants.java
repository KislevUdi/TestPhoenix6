package frc.robot;

import frc.robot.Util.TalonConfig;

public final class Constants {
  public static class ExampleConstants {
    public static final int MOTOR_ID = 2;
    public static final String CAN = "canivore";
    public static final double INCH = 0.0254;
    public static final double WHEEL_CIRC = 4 * INCH * Math.PI;
    public static final double GEAR_RATIO = 8.14;
    public static final double MOTOR_RATIO =  GEAR_RATIO / WHEEL_CIRC;

    public static final TalonConfig MOTOR_CONFIG = new TalonConfig(MOTOR_ID, CAN, "motor")
      .withBrake(false)
      .withCurrent(10,12,0.2)
      .withMotorRatio(MOTOR_RATIO)
      .withPID(0.01,0.001,0,0.5,0.12,0.01)
      .withVolts(5,-5);
  }


  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
  }
}