package frc.robot.Util;

/** Add your docs here. */
public class TalonConfig {
    public int id;
    public String canbus;
    public String name;
    public double maxVolt = 12;
    public double minVolt = -12;
    public double maxCurrent = 40;
    public double maxCurrentTreshold = 42;
    public double maxCurrentTriggerTime = 0.2;
    public double rampUpTime = 0.3;
    public boolean brake = true;
    public double motorRatio = 1;
    public boolean inverted = false;
    public closeLoopParam pid = new closeLoopParam(0.1,0.01,0,0.2,0.12,0.1);
    public double motionMagicAccel = 10;
    public double motionMagicVelocity = 1;
    public double motionMagicJerk = 10;

    class closeLoopParam {
        double kp;
        double ki;
        double kd;
        double ks;
        double kv;
        double ka;

        closeLoopParam(double kp, double ki, double kd, double ks, double kv, double ka) {
            this.ka = ka;
            this.kd = kd;
            this.ki = ki;
            this.kp = kp;
            this.ks = ks;
            this.kv = kv;
        }
    }

    public TalonConfig(int id, String canbus, String name) {
        this.id = id;
        this.canbus = canbus;
        this.name = name;
    }

    public TalonConfig withVolts(double maxVolt, double minVolt) {
        this.maxVolt = maxVolt;
        this.minVolt = minVolt;
        return this;
    }
    public TalonConfig withCurrent(double maxCurrent, double treshold, double trigerTime) {
        this.maxCurrent = maxCurrent;
        this.maxCurrentTreshold = treshold;
        this.maxCurrentTriggerTime = trigerTime;
        return this;
    }

    public TalonConfig withBrake(boolean brake) {
        this.brake = brake;
        return this;
    }
    public TalonConfig withInvert(boolean invert) {
        this.inverted = invert;
        return this;
    }
    public TalonConfig withRampTime(double rampTime) {
        this.rampUpTime = rampTime;
        return this;
    }
    public TalonConfig withMotorRatio(double ratio) {
        this.motorRatio = ratio;
        return this;
    }

    public TalonConfig withPID(double kp, double ki, double kd, double ks, double kv, double ka) {
        pid = new closeLoopParam(kp, ki, kd, ks, kv, ka);
        return this;
    }

}