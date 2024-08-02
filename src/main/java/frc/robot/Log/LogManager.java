package frc.robot.Log;

import java.util.ArrayList;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LogManager extends SubsystemBase {

    public static LogManager logManager;
    private DataLog log;
    private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTable table = ntInst.getTable("log");

    class LogEntry {
        DoubleLogEntry entry;
        Supplier<StatusSignal<Double>> getter;
        String name;
        DoublePublisher ntPublisher;

        LogEntry(String name,Supplier<StatusSignal<Double>> getter, boolean addToNT) {

            this.entry = new DoubleLogEntry(log, name);
            this.getter = getter;
            this.name = name;
            if(addToNT) {
                DoubleTopic dt = table.getDoubleTopic(name);
                ntPublisher = dt.publish();
            } else {
                ntPublisher = null;
            }
        }
    }

    ArrayList<LogEntry> logEntries = new ArrayList<>();

    public LogManager() {
        logManager = this;
        DataLogManager.start();
        log = DataLogManager.getLog();
    }

    private void add(String name,Supplier<StatusSignal<Double>> getter, boolean addToNT) {
        logEntries.add(new LogEntry(name, getter, addToNT));
    }

    public static void addEntry(String name,Supplier<StatusSignal<Double>> getter, boolean addToNT) {
        logManager.add(name, getter, addToNT);
    }

    @Override
    public void periodic() {
        super.periodic();
        for(LogEntry e : logEntries) {
            var st = e.getter.get();
            if(st.getStatus() == StatusCode.OK) {
                e.entry.append(st.getValue(), (long)(st.getTimestamp().getTime()*1000));
                if(e.ntPublisher != null) {
                    e.ntPublisher.set(st.getValue());
                }
            }
        }
    }
}
