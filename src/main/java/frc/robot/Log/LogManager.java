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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LogManager extends SubsystemBase {

    public static LogManager logManager; // singelton reference

    private DataLog log; //
    private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTable table = ntInst.getTable("log");

    /*
      class for a single data entry
     */
    class LogEntry {
        DoubleLogEntry entry;
        Supplier<StatusSignal<Double>> getterPhoenix6Status;
        Supplier<Double> getterDouble;
        String name;
        DoublePublisher ntPublisher;

        LogEntry(String name,Supplier<StatusSignal<Double>> getterPhoenix6Status,Supplier<Double> getterDouble, boolean addToNT) {

            this.entry = new DoubleLogEntry(log, name);
            this.getterPhoenix6Status = getterPhoenix6Status;
            this.getterDouble = getterDouble;
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
        DataLogManager.logNetworkTables(false);
        log = DataLogManager.getLog();
        DriverStation.startDataLog(log);
    }

    private void add(String name,Supplier<StatusSignal<Double>> getterPhoenix, Supplier<Double> getterDouble, boolean addToNT) { 
        logEntries.add(new LogEntry(name, getterPhoenix, getterDouble, addToNT));
    }

    public static void addEntry(String name,Supplier<StatusSignal<Double>> getterPhoenix, Supplier<Double> getterDouble, boolean addToNT) {
        logManager.add(name, getterPhoenix,getterDouble, addToNT);
    }

    @Override
    public void periodic() {
        super.periodic();
        double v;
        long time;
        for(LogEntry e : logEntries) {
            if(e.getterPhoenix6Status != null) {
               var st = e.getterPhoenix6Status.get();
                if(st.getStatus() == StatusCode.OK) {
                    v = st.getValue();
                    time = (long)(st.getTimestamp().getTime()*1000);
                } else {
                    continue;
                }
            } else {
                v = e.getterDouble.get();
                time = 0;
            }
            e.entry.append(v, time);
            if(e.ntPublisher != null) {
                e.ntPublisher.set(v);
            }
        }
    }
}
