// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class ExampleConstants {
    public static final int MOTOR_ID = 2;
    public static final String CAN = "canivore";
    public static final double INCH = 0.0254;
    public static final double WHEEL_CIRC = 4 * INCH * Math.PI;
    public static final double GEAR_RATIO = 8.14;
    public static final double MOTOR_RATIO = WHEEL_CIRC / GEAR_RATIO;

    public static final double KP = 0.01;
    public static final double KI = 0.001;
    public static final double KD = 0.0;
    public static final double KS = 0.5;
    public static final double KV = 0.11 * MOTOR_RATIO;
    public static final double KA = 0.01 * MOTOR_RATIO;
  }


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
