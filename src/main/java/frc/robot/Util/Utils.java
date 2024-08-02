package frc.robot.Util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.util.datalog.DoubleLogEntry;

public class Utils {

  public static double getDouble(StatusSignal<Double> st) {
    if(st.getStatus() == StatusCode.OK) {
      return st.getValue();
    } else {
      return 0;
    }
  }

    public static void logDouble(StatusSignal<Double> st, DoubleLogEntry entry) {
    if(st.getStatus() == StatusCode.OK) {
      entry.append(st.getValue(), (long)(st.getTimestamp().getTime()*1000));
    }
  }


}
