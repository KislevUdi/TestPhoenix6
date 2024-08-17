package frc.robot;

import frc.robot.Util.TalonConfig;

public final class Constants {
  public static class ExampleConstants {
    public static final int DRIVE_MOTOR_ID = 2;
    public static final int STEER_MOTOR_ID = 8;
    public static final String CANBUS = "canivore";
    public static final double INCH = 0.0254;
    public static final double WHEEL_CIRC = 4 * INCH * Math.PI;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double DRIVE_MOTOR_RATIO =  DRIVE_GEAR_RATIO / WHEEL_CIRC; // motor rotaion to meter
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double STEER_MOTOR_RATIO =  STEER_GEAR_RATIO / 360; // steer rotation to degrees

    public static final TalonConfig DRIVE_MOTOR_CONFIG = 
      new TalonConfig(DRIVE_MOTOR_ID, CANBUS, "drive motor")
      .withBrake(false)
      .withCurrent(10,12,0.2)
      .withMotorRatio(DRIVE_MOTOR_RATIO)
      .withPID(0.01,0.001,0,0.5,0.12,0.01,0)
      .withVolts(5,-5)
      .withInvert(false)
      .withRampTime(0.3)
      .withMotionMagic(2, 6, 10);

    public static final TalonConfig STEER_MOTOR_CONFIG = 
      new TalonConfig(STEER_MOTOR_ID, CANBUS, "steer motor")
      .withBrake(false)
      .withCurrent(10,12,0.2)
      .withMotorRatio(STEER_MOTOR_RATIO)
      .withPID(0.01,0.001,0,0.5,0.12,0.01,0)
      .withVolts(5,-5)
      .withInvert(false)
      .withRampTime(0.3)
      .withMotionMagic(360, 600, 1000);
  }


  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
  }
}