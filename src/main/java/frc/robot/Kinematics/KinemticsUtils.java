package frc.robot.Kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.math.Pair;

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

    /**
     * Return the center change in a cycle using the acceleromator with last velocity and delta time
     * 
     * @param lastVelocity
     * @param acceleration
     * @param deltaTime
     * @return Pair - Translation2D displacement, Translation2D new velocity vector
     */
    public static Pair<Translation2d, Translation2d> centerMoveByAcceleration(Translation2d lastVelocity, Translation2d acceleration, double deltaTime) {
        return new Pair<Translation2d, Translation2d>(lastVelocity.times(deltaTime).plus(acceleration.times(deltaTime*deltaTime/2)), lastVelocity.plus(acceleration.times(deltaTime)));
    }

    /**
     * Return the center change using a module displacement, heading change and vector from module to center
     * 
     * @param moduleChange
     * @param gyroChange
     * @param moduleToCenter
     * @return center displacement
     */
    public static Translation2d centerMoveByModuleAndGyro(Translation2d moduleChange, Rotation2d gyroChange, Translation2d moduleToCenter) {
        return moduleChange.plus(moduleToCenter.rotateBy(gyroChange)).minus(moduleToCenter);
    }

    /**
    * Return the center change using 2 modules change and vectors from first module to center and to second module
    * 
    * @param module1Change
    * @param module2Change
    * @param module1ToCenter
    * @param module1to2
    * @return Pair - robot change and the error of the length between the 2 modules after the change
    */
    public static Pair<Translation2d, Double> centerMoveByModulesChange(Translation2d module1Change, Translation2d module2Change, Translation2d module1ToCenter, Translation2d module1to2) {
        Translation2d m12 = module2Change.plus(module1to2).minus(module1Change);
        double lengthErrror = m12.getNorm() - module1to2.getNorm();
        Rotation2d rotation = m12.getAngle().minus(module1to2.getAngle());
        return new Pair<Translation2d, Double>(module1Change.plus(module1ToCenter.rotateBy(rotation)).minus(module1ToCenter),lengthErrror);
    }
}
