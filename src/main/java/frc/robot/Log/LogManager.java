package frc.robot.Log;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class LogManager extends SubsystemBase {

    public static LogManager logManager; // singelton reference

    public static int logLevel = 0;

    private DataLog log; //
    private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTable table = ntInst.getTable("log");

    /*
     * class for a single data entry
     */
    public static class LogEntry {
        static LogManager manager;
        DoubleLogEntry entry;  // wpilib log entry
        Supplier<StatusSignal<Double>> getterPhoenix6Status; // supplier of phoenix 6 status signal
        DoubleSupplier getterDouble; // supplier for double data - if no status signal provider
        BiConsumer<Double, Long> consumer = null; // optional consumer when data changed - data value and time
        String name;
        DoublePublisher ntPublisher;   // network table punlisher
        double lastValue = Double.MAX_VALUE; // last value - only logging when value changes
        int level = 1;
        boolean skipAutoLog = false;

        /*
         * Constructor with the suppliers and boolean if add to network table
         */
        LogEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix6Status, DoubleSupplier getterDouble,
                boolean addToNT, int level) {
            this.entry = new DoubleLogEntry(manager.log, name);
            this.getterPhoenix6Status = getterPhoenix6Status;
            this.getterDouble = getterDouble;
            this.name = name;
            if (addToNT) {
                DoubleTopic dt = manager.table.getDoubleTopic(name);
                ntPublisher = dt.publish();
            } else {
                ntPublisher = null;
            }
            skipAutoLog = getterPhoenix6Status == null && getterDouble == null;
            this.level = level;
        }
        LogEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix6Status, boolean addToNT, int level) {
            this(name, getterPhoenix6Status, null, addToNT, level);
        }
        LogEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix6Status) {
            this(name, getterPhoenix6Status, null, true, 5);
        }
        LogEntry(String name, DoubleSupplier getterDouble,boolean addToNT, int level) {
            this(name, null, getterDouble, addToNT, level);
        }
        LogEntry(String name, DoubleSupplier getterDouble) {
            this(name, null, getterDouble, true, 5);
        }

        /*
         * perform a periodic log
         * get the data from the getters and call the actual log
         */
        void log() {
            if(skipAutoLog || level < logLevel)
                return;

            double v;
            long time = 0;

            if (getterPhoenix6Status != null) {
                var st = getterPhoenix6Status.get();
                if (st.getStatus() == StatusCode.OK) {
                    v = st.getValue();
                    time = (long) (st.getTimestamp().getTime() * 1000);
                } else {
                    v = 1000000 + st.getStatus().value;
                }
            } else {
                v = getterDouble.getAsDouble();
                time = 0;
            }
            log(v, time);
        }

        /*
         * log a value use zero (current) time
         */
        public void log(double v) {
            log(v, 0);
        }

        /*
         * Log data and time if data changed
         * also publish to network table (if required)
         * also call consumer if set
         */
        public void log(double v, long time) {
            if (v != lastValue) {
                entry.append(v, time);
                if (ntPublisher != null) {
                    ntPublisher.set(v);
                }
                if(consumer != null) {
                    consumer.accept(v, time);
                }
                lastValue = v;
            }
        }

        // set the consumer
        public void setConsumer(BiConsumer<Double,Long> consumer) {
            this.consumer = consumer;
        }

        public void setLevel(int level) {
            this.level = level;
        }
    }

    // array of log entries
    ArrayList<LogEntry> logEntries = new ArrayList<>();

    // Log managerconstructor
    public LogManager() {
        logManager = this;
        LogEntry.manager = this;
        DataLogManager.start();
        DataLogManager.logNetworkTables(false);
        log = DataLogManager.getLog();
        DriverStation.startDataLog(log);
    }

    /*
     * add a log entry with all data
     */
    private LogEntry add(String name, Supplier<StatusSignal<Double>> getterPhoenix, DoubleSupplier getterDouble,
            boolean addToNT, int level) {
        LogEntry entry = new LogEntry(name, getterPhoenix, getterDouble, addToNT, level);
        logEntries.add(entry);
        return entry;
    }

    /*
     * get a log entry - if not found, creat one
     */
    private LogEntry get(String name, boolean addToNT) {
        LogEntry e = find(name);
        if(e != null) {
            return e;
        }
        return new LogEntry(name, null, null, addToNT,5);
    }

    /*
     * find a log entry by name
     */
    private LogEntry find(String name) {
        for(LogEntry e : logEntries) {
            if(name.equals(e.name)) {
                return e;
            }
        }
        return null;
    }

    /*
     * Static function - add log entry with all data
     */
    public static LogEntry addEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix,boolean addToNT, int level) {
        return logManager.add(name, getterPhoenix, null, addToNT, level);
    }
    /*
     * Static function - add log entry for double supplier with option to add to network table
     */
    public static LogEntry addEntry(String name, DoubleSupplier getterDouble, boolean addToNT, int level) {
        return logManager.add(name, null, getterDouble, addToNT, level);
    }
    /*
     * Static function - add log entry for status signal with network table
     */
    public static LogEntry addEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix) {
        return logManager.add(name, getterPhoenix, null, true, 5);
    }
    /*
     * Static function - add log entry for double supplier with network table
     */
    public static LogEntry addEntry(String name, DoubleSupplier getterDouble) {
        return logManager.add(name, null, getterDouble, true, 5);
    }
    /*
     * Static function - get an entry, create if not foune - will see network table is crating new
     */
    public static LogEntry getEntry(String name, boolean addToNT) {
        return logManager.get(name, addToNT);
    }
    /*
     * Static function - get an entry, create if not foune - will see network table is crating new
     */
    public static LogEntry getEntry(String name) {
        return logManager.get(name, true);
    }

    /*
     * Log text message - also will be sent System.out
     */
    public static void log(String message) {
        DataLogManager.log(message);
    }

    /**
     * Get an array of entries - all under a base name
     * @param baseName
     * @param names
     * @return
     */
    public static LogEntry[] getLogEntries(String baseName, String...names) {
        LogEntry[] entries = new LogEntry[names.length];
        for(int i = 0; i < names.length; i++) {
            entries[i] = getEntry(baseName + "/" + names[i]);
        }
        return entries;
    }

    /**
     * Log values for an array of entries
     * @param entries
     * @param time
     * @param values
     */
    public static void log(LogEntry[] entries, long time, double...values) {
        for(int i = 0; i < entries.length; i++) {
            entries[i].log(values[i], time);
        }
    }
    /**
     * Log values for an array of entries - using current time
     * @param entries
     * @param values
     */
    public static void log(LogEntry[] entries, double...values) {
        log(entries,time(),values);
    }

    public static long time() {
        return (long)(Timer.getFPGATimestamp()*1000);
    }

    @Override
    public void periodic() {
        super.periodic();
        for (LogEntry e : logEntries) {
            e.log();
        }
    }
}
