package frc.robot.Log;

import java.util.ArrayList;
import java.util.function.Supplier;

import javax.swing.LayoutStyle;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LogManager extends SubsystemBase {

    public static LogManager logManager; // singelton reference

    private DataLog log; //
    private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTable table = ntInst.getTable("log");

    /*
     * class for a single data entry
     */
    public class LogEntry {
        DoubleLogEntry entry;
        Supplier<StatusSignal<Double>> getterPhoenix6Status;
        Supplier<Double> getterDouble;
        String name;
        DoublePublisher ntPublisher;
        double lastValue = Double.MAX_VALUE;

        LogEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix6Status, Supplier<Double> getterDouble,
                boolean addToNT) {

            this.entry = new DoubleLogEntry(log, name);
            this.getterPhoenix6Status = getterPhoenix6Status;
            this.getterDouble = getterDouble;
            this.name = name;
            if (addToNT) {
                DoubleTopic dt = table.getDoubleTopic(name);
                ntPublisher = dt.publish();
            } else {
                ntPublisher = null;
            }
        }

        public void log() {
            double v;
            long time = 0;

            if (getterPhoenix6Status != null) {
                var st = getterPhoenix6Status.get();
                if (st.getStatus() == StatusCode.OK) {
                    v = st.getValue();
                    time = (long) (st.getTimestamp().getTime() * 1000);
                } else {
                    v = lastValue;
                }
            } else {
                v = getterDouble.get();
                time = 0;
            }
            log(v, time);
        }

        public void log(double v) {
            log(v, 0);
        }

        public void log(double v, long time) {
            if (v != lastValue) {
                entry.append(v, time);
                if (ntPublisher != null) {
                    ntPublisher.set(v);
                }
                lastValue = v;
            }
        }
    }

    ArrayList<LogEntry> logEntries = new ArrayList<>();

    public LogManager() {
        logManager = this;
        DataLogManager.start();
        DataLogManager.logNetworkTables(false);
        log = DataLogManager.getLog();
        DriverStation.startDataLog(log);
    }

    private void add(String name, Supplier<StatusSignal<Double>> getterPhoenix, Supplier<Double> getterDouble,
            boolean addToNT) {
        logEntries.add(new LogEntry(name, getterPhoenix, getterDouble, addToNT));
    }

    private LogEntry get(String name, boolean addToNT) {
        return new LogEntry(name, null, null, addToNT);
    }

    public static void addEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix,
            Supplier<Double> getterDouble, boolean addToNT) {
        logManager.add(name, getterPhoenix, getterDouble, addToNT);
    }

    public static LogEntry getEntry(String name, boolean addToNT) {
        return logManager.get(name, addToNT);
    }

    @Override
    public void periodic() {
        super.periodic();
        for (LogEntry e : logEntries) {
            e.log();
        }
    }
}
