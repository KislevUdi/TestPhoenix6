package frc.robot.Sysid;

import java.util.ArrayList;
import java.util.function.DoubleConsumer;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.Log.LogManager;
import frc.robot.Log.LogManager.LogEntry;

public class Sysid {
    /** 
     * analyze data from log - arrays of data
     * For Talon parameters:
     * - Velocity - kv, ks, ka
     * - Position - Motion Magic
     *    - Elevator - add kg
     *    - arm - add k sin - will be used to calculate the feedforward to target position
     * 
     * Normalize data to same time - volt, position, velocity, acceleration
     * create the matrix - with optional kg and k-sin
     * ignore zero volt/power
     * for k-sin - use a posToRad parameter
     * calculate the gains
     * check error - max abs error, avg error squared, std
     * log - number of data, gains, errors, min velocity, max velocity
     * check error for range - 20% of low velocity, 20-50%, 50-100%
     * if std/max error/avg err2 are smaller for region - recalculate for affected range
     * set kp = 10*ka, ki = 2*kv
     * 
     * 
    */
    String motorName;
    LogEntry voltsEntry;
    LogEntry positionEntry;
    LogEntry velocityEntry;
    LogEntry acceleratioEntry;
    boolean useKG = false;
    boolean useKSIN = false;
    boolean useKV2 = false;
    double posToRad = 0;
    double minVelocityToUse = 0;
    double maxVelocityToUse = Double.MAX_VALUE;
    boolean calcPerDirection = false;

    double lastPosition = 0;
    double lastVelocity = 0;
    double lastAcceleration = 0;

    ArrayList<timedData> data = new ArrayList<>();

    class timedData {
        double volts;
        double position;
        double velocity;
        double accelearation;
        long time;

        public timedData(double volts, double position, double velocity, double accelearation, long time) {
            this.volts = volts;
            this.position = position;
            this.velocity = velocity;
            this.accelearation = accelearation;
            this.time = time;
        }

    }


    public Sysid(String motorName) {
        this.motorName = motorName;
        getEntries();
    }

    private void getEntries() {
        positionEntry = LogManager.getEntry(motorName + "/position");
        velocityEntry = LogManager.getEntry(motorName + "/Velocity");
        acceleratioEntry = LogManager.getEntry(motorName + "/Acceleration");
        voltsEntry = LogManager.getEntry(motorName + "/Voltage");
    }

    public void startCollecting() {
        positionEntry.setConsumer(this::positionConsumer);
        velocityEntry.setConsumer(this::velocityConsumer);
        acceleratioEntry.setConsumer(this::accelerationConsumer);
        voltsEntry.setConsumer(this::voltsConsumer);

    }
    public void stopCollecting() {
        positionEntry.setConsumer(null);
        velocityEntry.setConsumer(null);
        acceleratioEntry.setConsumer(null);
        voltsEntry.setConsumer(null);

    }

    public Sysid withKG() {
        useKG = true;
        return this;
    }
    public Sysid withKV2() {
        useKV2 = true;
        return this;
    }
    public Sysid withKSin(double posToRad) {
        useKSIN = true;
        this.posToRad = posToRad;
        return this;
    }
    public Sysid withVelocities(double minVelocityToUse, double maxVelocityToUse) {
        this.minVelocityToUse = minVelocityToUse;
        this.maxVelocityToUse = maxVelocityToUse;
        return this;
    }
    public Sysid withCalcPerDirection() {
        calcPerDirection = true;
        return this;
    }

    public void voltsConsumer(double volts, long time) {
        data.add(new timedData(volts, lastPosition, lastVelocity, lastAcceleration, time));
    }
    public void velocityConsumer(double velocity, long time) {
        lastVelocity = velocity;
    }
    public void positionConsumer(double position, long time) {
        lastPosition = position;
    }
    public void accelerationConsumer(double accelearation, long time) {
        lastAcceleration = accelearation;
    }

    public void calculate() {
        stopCollecting();
        if(calcPerDirection) {
            calculateForRange(minVelocityToUse, 0, false);
            calculateForRange(0, maxVelocityToUse, false);
        } else {
            calculateForRange(minVelocityToUse, maxVelocityToUse, true);
        }
    }

    private void calculateForRange(double min, double max, boolean splitIfNeeded) {
        ArrayList<timedData> d = inRange(min, max);
        if(d.size() < 10) {
            LogManager.log("Sysid for " + motorName + " range " + min + "-" + max + " - not enough data, only " + d.size() + " cases");
            return;
        }
        int nColumns = 3;
        if(useKG) nColumns++;
        if(useKV2) nColumns++;
        if(useKSIN) nColumns++;
        SimpleMatrix m = new SimpleMatrix(d.size(), nColumns);
        SimpleMatrix r = new SimpleMatrix(d.size(),1);
        int row = 0;
        for(timedData td : d) {
            fillMatrix(m, r, row++, td);
        }
        SimpleMatrix res = m.solve(r);
        double ks = r.get(0, 0);
        double kv = r.get(1, 0);
        double ka = r.get(2, 0);
        row = 3;
        double kg = useKG? r.get(row++,0):0;
        double kvw = useKV2? r.get(row++,0):0;
        double ksin = useKSIN? r.get(row++,0):0;
        double kd = 0;
        SimpleMatrix caclulatedVolt = m.mult(res);
        SimpleMatrix errors = r.minus(caclulatedVolt);
        double maxError = errors.elementMaxAbs();
        double sumError = errors.elementSum();
        double sumErrorSquared = (errors.transpose().mult(errors)).elementSum();
        double n = errors.getNumRows();
        double avgErrorSquared = sumErrorSquared / n;
        double avgError = sumError / n;
        SimpleMatrix vels = m.getColumn(1);
        double minVel = vels.elementMin();
        double maxVel = vels.elementMax();
        Pair<Double,Double> kpi = calculatePID(kv, ka, maxVel-minVel);
        double kp = kpi.getFirst();
        double ki = kpi.getSecond();
        log(minVel, maxVel,ks, kv, ka, kg, kvw, ksin, kp, ki, kd, maxError, avgError, avgErrorSquared);
        if(splitIfNeeded && (maxError > 0.1 || avgErrorSquared > 0.01)) {
            double range = maxVel - minVel;
            calculateForRange(minVel, minVel + 0.2*range, false);
            calculateForRange(minVel + 0.2*range, minVel + 0.6*range, false);
            calculateForRange(minVel + 0.6*range, maxVel, false);
        }
    }

    private Pair<Double,Double> calculatePID(double kv, double ka, double velRange) {
        if(ka < 1E-7) {
            return new Pair<Double,Double>(ka*10,kv/2);
        }
        var sys = LinearSystemId.identifyVelocitySystem(kv, ka);
        var lqr = new LinearQuadraticRegulator<>(sys, VecBuilder.fill(velRange*0.1),VecBuilder.fill(8),Robot.kDefaultPeriod);
        lqr.latencyCompensate(sys, Robot.kDefaultPeriod, Robot.kDefaultPeriod-0.01);
        var k = lqr.getK();
        return new Pair<Double,Double>(k.get(0, 0), k.get(0,1));
    }

    private void fillMatrix(SimpleMatrix mat, SimpleMatrix r, int row, timedData d) {
        r.set(row, 0, d.volts);
        mat.set(row,0,Math.signum(d.velocity));
        mat.set(row,1,d.velocity);
        mat.set(row,2,d.accelearation);
        int col = 3;
        if(useKG) {
            mat.set(row,col++,1);
        }
        if(useKV2) {
            mat.set(row,col++,Math.signum(d.velocity)*d.velocity*d.velocity);
        }
        if(useKSIN) {
            mat.set(row,col++,Math.sin(d.position*posToRad));
        }
    }

    private ArrayList<timedData> inRange(double min, double max) {
        ArrayList<timedData> res = new ArrayList<>();
        for(timedData d : data) {
            if(d.velocity > min && d.velocity < max && d.volts != 0) {
                res.add(d);
            }
        }
        return res;
    }

    private void log(double min, double max, double ks, double kv, double ka, double kg, double kv2, double ksin, double kp, double ki, double kd, double maxError, double avgError, double avgErrorSquared) {
        String name = "Sysid/" + motorName + "/" + min + "-" + max + "/";
        long time = RobotController.getFPGATime();
        LogManager.getEntry(name + "KS").log(ks, time);
        LogManager.getEntry(name + "KV").log(kv, time);
        LogManager.getEntry(name + "KA").log(ka, time);
        LogManager.getEntry(name + "KG").log(kg, time);
        LogManager.getEntry(name + "KV2").log(kv2, time);
        LogManager.getEntry(name + "KSIN").log(ksin, time);
        LogManager.getEntry(name + "KP").log(kp, time);
        LogManager.getEntry(name + "KI").log(ki, time);
        LogManager.getEntry(name + "KD").log(kd, time);
        LogManager.getEntry(name + "maxError").log(maxError, time);
        LogManager.getEntry(name + "avgError").log(avgError, time);
        LogManager.getEntry(name + "avgErrorSquared").log(avgErrorSquared, time);
    }

    public Command getCommand(DoubleConsumer setPower, double minPower, double maxPower, double accelearationTime, Subsystem subsystem) {
        return new SysidCommand(minPower, maxPower, accelearationTime, setPower, this, subsystem);
    }
    
    
}
