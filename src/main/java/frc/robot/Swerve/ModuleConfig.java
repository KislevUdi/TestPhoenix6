package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Util.TalonConfig;

public class ModuleConfig {
    public String name;
    public TalonConfig driveConfig;
    public TalonConfig steerConfig;
    public int absEncoderID;
    public String absEncodedrCanbus = "rio";
    public double absEncoderOffsetInDegrees;
    public Translation2d positionRelativeToRobotCenter;
}
