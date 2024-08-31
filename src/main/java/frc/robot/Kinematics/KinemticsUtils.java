package frc.robot.Kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

/** Add your docs here. */
public class KinemticsUtils {

    public static BuiltInAccelerometer builtInAccelerometer = new BuiltInAccelerometer();

    public static Translation2d curveTranslation(double distance, Rotation2d startAngle, Rotation2d endAngle) {
        return new Translation2d(distance, startAngle.plus(endAngle).div(2));
    }

    public static Translation2d offset(Translation2d t1, Translation2d t2, Translation2d p1ToP2Vector, Rotation2d startAngle, Rotation2d endAngle) {
        return p1ToP2Vector.plus(t2).minus(p1ToP2Vector.rotateBy(endAngle.minus(startAngle))).minus(t1);
    }

    public static double accel() {
        return Math.hypot(builtInAccelerometer.getX(), builtInAccelerometer.getY());
    }
}
